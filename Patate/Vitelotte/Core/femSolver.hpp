/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "femSolver.h"


namespace Vitelotte
{


template < class _Mesh, class _ElementBuilder >
FemSolver<_Mesh, _ElementBuilder>::FemSolver(Mesh* _mesh, const ElementBuilder& elementBuilder)
  : m_mesh(_mesh),
    m_elementBuilder(elementBuilder),
    m_solved(false)
{
}


template < class _Mesh, class _ElementBuilder >
void
FemSolver<_Mesh, _ElementBuilder>::build()
{
    unsigned nCoefficients = 0;
    m_elementBuilder.resetStatus();
    m_solved = false;
    for(FaceIterator elem = m_mesh->facesBegin();
        elem != m_mesh->facesEnd(); ++elem)
    {
        nCoefficients += m_elementBuilder.nCoefficients(*m_mesh, *elem);
    }

    TripletVector coefficients(nCoefficients);
    TripletVectorIterator it = coefficients.begin();
    for(FaceIterator elem = m_mesh->facesBegin();
        elem != m_mesh->facesEnd(); ++elem)
    {
        m_elementBuilder.addCoefficients(it, *m_mesh, *elem);

        if(m_elementBuilder.status() == ElementBuilder::STATUS_ERROR)
        {
            return;
        }
    }
    assert(it == coefficients.end());

    m_stiffnessMatrix.resize(m_mesh->nNodes(), m_mesh->nNodes());
    m_stiffnessMatrix.setFromTriplets(
                coefficients.begin(), coefficients.end());

    sort();
}


template < class _Mesh, class _ElementBuilder >
void
FemSolver<_Mesh, _ElementBuilder>::sort()
{
    if(m_elementBuilder.status() == ElementBuilder::STATUS_ERROR)
    {
        return;
    }

    int n = m_stiffnessMatrix.cols();
    m_perm.resize(n);

    m_ranges.clear();


    Eigen::Matrix<bool, Eigen::Dynamic, 1> mask(n);
    mask.fill(true);
    Eigen::VectorXi stack(n);

    int sPos = 0;   // search position
    int count = 0;  // nb items in stack
    int nUnk = 0;   // nb found unknown nodes

    while(sPos < n && m_mesh->isConstraint(Node(sPos))) ++sPos;
    if(sPos < n)
    {
        m_ranges.push_back(nUnk);
        stack[count++] = sPos;
        mask[sPos] = false;
    }

    while(count > 0)
    {
        int col = stack[--count];
        m_perm[nUnk++] = col;

        for(typename StiffnessMatrix::InnerIterator it(m_stiffnessMatrix, col); it ;++it)
        {
            if(mask[it.index()] && !m_mesh->isConstraint(Node(it.index())))
            {
                stack[count++] = it.index();
                mask[it.index()] = false;
            }
        }

        if(count == 0)
        {
            while(sPos < n && (!mask[sPos] || m_mesh->isConstraint(Node(sPos)))) ++sPos;
            if(sPos < n)
            {
                m_ranges.push_back(nUnk);
                stack[count++] = sPos;
                mask[sPos] = false;
            }
        }
    }

    m_ranges.push_back(nUnk);

    for(int col = 0; col < n; ++col)
    {
        assert(m_mesh->isConstraint(Node(col)) == mask[col]);
        if(mask[col])
            m_perm[nUnk++] = col;
    }
}


template < class _Mesh, class _ElementBuilder >
void
FemSolver<_Mesh, _ElementBuilder>::solve()
{
    if(m_elementBuilder.status() == ElementBuilder::STATUS_ERROR)
    {
        return;
    }

    // Apply permutation
    Eigen::PermutationMatrix<Eigen::Dynamic> perm(m_perm), permInv(perm.inverse());
    StiffnessMatrix mat;
    mat/*.template selfadjointView<Eigen::Lower>()*/ =
            m_stiffnessMatrix.template selfadjointView<Eigen::Lower>().twistedBy(permInv);
//            m_stiffnessMatrix.twistedBy(permInv);

    // compute RHS
    unsigned nUnknowns = m_ranges.back();
    unsigned nConstraints = m_stiffnessMatrix.cols() - nUnknowns;
    Matrix constraints(nConstraints, (unsigned)Mesh::Chan);

    // As sort() keep ordering, constraints will have the right indices.
    unsigned count = 0;
    for(unsigned i = 0; i < m_mesh->nNodes(); ++i)
    {
        if(m_mesh->isConstraint(Node(i)))
        {
            constraints.row(count++) =
                    m_mesh->nodeValue(Node(i)).template cast<Scalar>();
        }
    }
    assert(count == nConstraints);

    m_x.resize(nUnknowns, Mesh::Chan);
    m_x.setZero();

//    Eigen::Matrix<float, 4, 2> g;
//    g << 1., 0,
//         0, 1.,
//         0, 0,
//         0, 0;
////    g.setZero();

//    typename Mesh::Vertex vert(0);
//    typename Mesh::HalfedgeAroundVertexCirculator hit = m_mesh->halfedges(vert);
//    typename Mesh::HalfedgeAroundVertexCirculator end = hit;
//    do
//    {
//        Eigen::Vector2f e = (m_mesh->position(m_mesh->toVertex(*hit))
//                - m_mesh->position(m_mesh->fromVertex(*hit))).normalized();
//        Eigen::Vector4f v = g * e;

//        unsigned ni = m_mesh->edgeGradientNode(*hit).idx();
//        std::cerr << "e " << ni << ": " << e.transpose() << " -> " << v.transpose() << "\n";

//        m_x.row(ni) = v.cast<Scalar>();

//        ++hit;
//    } while(hit != end);

    m_x -= mat.topRightCorner(nUnknowns, nConstraints) * constraints;

    unsigned nbRanges = m_ranges.size()-1;

#ifdef _OPENMP
#pragma omp parallel for schedule(static,1)
#endif
    for(unsigned k = 0; k < nbRanges; ++k)
    {
        int start = m_ranges[k];
        int size  = m_ranges[k+1]-start;
        StiffnessMatrix L(size,size);
        L.reserve(size*20);

        for(int j = 0; j < size; ++j)
        {
            L.startVec(j);
            for(typename StiffnessMatrix::InnerIterator it(mat, start + j); it; ++it)
            {
//                if(it.index() >= nUnknowns) continue;
                if(it.index() < j || it.index() >= nUnknowns) continue;
                L.insertBackByOuterInnerUnordered(j, it.index() - start) = it.value();
            }
        }

        L.finalize();

        std::cout << "range: " << start << " (" << m_perm[start] << "), " << size << "\n";

        Eigen::SimplicialLDLT<StiffnessMatrix> solver(L);

//        Eigen::BiCGSTAB<StiffnessMatrix> solver;
//        solver.compute(L);
//        std::cout << "BiCGSTAB: " << solver.iterations() << " iters, error = " << solver.error() << "\n";

//        Eigen::SparseLU<StiffnessMatrix> solver;
//        Eigen::SparseQR<StiffnessMatrix, Eigen::COLAMDOrdering<int> > solver(L);
//        solver.analyzePattern(L);
//        solver.factorize(L);

        m_x.middleRows(start,size) = solver.solve(m_x.middleRows(start, size));

        switch(solver.info()) {
        case Eigen::Success: std::cout << "Success\n"; break;
        case Eigen::NumericalIssue: std::cout << "NumericalIssue\n"; break;
        case Eigen::NoConvergence: std::cout << "NoConvergence\n"; break;
        case Eigen::InvalidInput: std::cout << "InvalidInput\n"; break;
        }
    }

    for(unsigned i = 0; i < nUnknowns; ++i)
    {
        assert(!m_mesh->isConstraint(Node(m_perm[i])));
        m_mesh->nodeValue(Node(m_perm[i])) = m_x.row(i).
                    template cast<typename Mesh::Scalar>();
    }

    m_solved = true;
}


}  // namespace Vitelotte
