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

    typedef typename Mesh::NodeValue NodeValue;

    Eigen::VectorXi iperm(m_perm.rows());
    for(int i=0; i<m_perm.rows(); ++i)
        iperm(m_perm(i)) = i;
    for(typename Mesh::HalfedgeIterator hit = m_mesh->halfedgesBegin();
        hit != m_mesh->halfedgesEnd(); ++hit) {

        if(m_mesh->nVertexGradientConstraints(*hit) == 0)
            continue;

        typename Mesh::Vertex from = m_mesh->fromVertex(*hit);
        typename Mesh::Vertex to   = m_mesh->  toVertex(*hit);
        std::cout << "Found constrained he: " <<
                     (m_mesh->position(to) - m_mesh->position(from)).transpose() << "\n";
        Node n = m_mesh->vertexGradientDummyNode(*hit);
        if(n.isValid()) {
            bool v0c = m_mesh->isGradientConstraint(from);
            const typename Mesh::Gradient& grad = m_mesh->gradientConstraint(v0c? from: to);
            std::cout << grad << "\n";
            typename Mesh::Vector v = m_mesh->position(to) - m_mesh->position(from);
            if(!v0c) v = -v;
            NodeValue cons = grad * v;
            std::cout << "set rhs " << n.idx() << ": " << cons.transpose() << "\n";
            m_x.row(iperm(n.idx())) = cons.template cast<Scalar>();
        }
    }
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

        // Skip unused, isolated nodes
        if(size == 1 && mat.coeffRef(start, start) == 0)
            continue;

        StiffnessMatrix L(size,size);
        L.reserve(size*20);

        for(int j = 0; j < size; ++j)
        {
            L.startVec(j);
            for(typename StiffnessMatrix::InnerIterator it(mat, start + j); it; ++it)
            {
                if(it.index() >= nUnknowns) continue;
//                if(it.index() < j || it.index() >= nUnknowns) continue;
                L.insertBackByOuterInnerUnordered(j, it.index() - start) = it.value();
            }
        }

        L.finalize();

        std::cout << "range: " << start << " (" << m_perm[start] << "), " << size << "\n";

//        Eigen::SimplicialLDLT<StiffnessMatrix> solver(L);

//        Eigen::BiCGSTAB<StiffnessMatrix> solver;
//        solver.compute(L);

        Eigen::SparseLU<StiffnessMatrix> solver;
//        Eigen::SparseQR<StiffnessMatrix, Eigen::COLAMDOrdering<int> > solver(L);
        solver.analyzePattern(L);
        solver.factorize(L);

        m_x.middleRows(start,size) = solver.solve(m_x.middleRows(start, size));

//        std::cout << "BiCGSTAB: " << solver.iterations() << " iters, error = " << solver.error() << "\n";

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

//    typedef typename Mesh::NodeValue NodeValue;
//    typename Mesh::template VertexProperty<bool> isGc =
//            m_mesh->template getVertexProperty<bool>("v:isGradientConstraint");

//    for(typename Mesh::FaceIterator fit = m_mesh->facesBegin();
//        fit != m_mesh->facesEnd(); ++fit)
//    {
//        typename Mesh::HalfedgeAroundFaceCirculator hit = m_mesh->halfedges(*fit);
//        typename Mesh::HalfedgeAroundFaceCirculator hend = hit;
//        do ++hit;
//        while(!isGc[m_mesh->toVertex(*hit)] && hit != hend);
//        bool isPgc = isGc[m_mesh->toVertex(*hit)];
//        if(!isPgc)
//            continue;

//        typename Mesh::Edge e2 = m_mesh->edge(*hit);
//        typename Mesh::Halfedge h2 = *hit;
//        ++hit;
//        typename Mesh::Edge e1 = m_mesh->edge(*hit);
//        typename Mesh::Halfedge h1 = *hit;
//        --hit;

//        Eigen::Matrix<Scalar, 4, 9> nodes;
////        bool orient[3];
//        typename ElementBuilder::Vector p[3];
//        --hit;
//        for(int i = 0; i < 3; ++i)
//        {
////            orient[i] = m_mesh->halfedgeOrientation(*hit);
//            nodes.col(3+i) = m_mesh->nodeValue(m_mesh->edgeValueNode(*hit)).template cast<Scalar>();
//            nodes.col(6+i) = m_mesh->nodeValue(m_mesh->edgeGradientNode(*hit)).template cast<Scalar>()
//                    * (m_mesh->halfedgeOrientation(*hit)? 1: -1);
//            ++hit;
//            nodes.col(i) = m_mesh->nodeValue(m_mesh->toVertexValueNode(*hit)).template cast<Scalar>();
//            p[i] = m_mesh->position(m_mesh->toVertex(*hit)).template cast<Scalar>();
//        }

//        FVElement<Scalar> elem(p);

//        typedef Eigen::Matrix<Scalar, 9, 1> Vector9;
//        Vector9 fde1, fde2;
//        fde1 <<
//            -1.0L/2.0L*(elem.doubleArea()*(2*elem.dldn(0, 1) + elem.dldn(1, 1)) + 7*elem.edgeLength(1))/(elem.edgeLength(1)*elem.edgeLength(2)),
//            (1.0L/2.0L)*(elem.doubleArea()*(elem.dldn(0, 0) + 2*elem.dldn(1, 0)) - elem.edgeLength(0))/(elem.edgeLength(0)*elem.edgeLength(2)),
//            -1.0L/2.0L*elem.doubleArea()*(elem.edgeLength(0)*(elem.dldn(1, 1) + 2*elem.dldn(2, 1)) - elem.edgeLength(1)*(elem.dldn(0, 0) + 2*elem.dldn(2, 0)))/(elem.edgeLength(0)*elem.edgeLength(1)*elem.edgeLength(2)),
//            -4/elem.edgeLength(2),
//            4/elem.edgeLength(2),
//            4/elem.edgeLength(2),
//            elem.doubleArea()/(elem.edgeLength(0)*elem.edgeLength(2)),
//            -elem.doubleArea()/(elem.edgeLength(1)*elem.edgeLength(2)),
//            0;
//        fde2 <<
//            -1.0L/2.0L*(elem.doubleArea()*(2*elem.dldn(0, 2) + elem.dldn(2, 2)) + 7*elem.edgeLength(2))/(elem.edgeLength(1)*elem.edgeLength(2)),
//            -1.0L/2.0L*elem.doubleArea()*(elem.edgeLength(0)*(2*elem.dldn(1, 2) + elem.dldn(2, 2)) - elem.edgeLength(2)*(elem.dldn(0, 0) + 2*elem.dldn(1, 0)))/(elem.edgeLength(0)*elem.edgeLength(1)*elem.edgeLength(2)),
//            (1.0L/2.0L)*(elem.doubleArea()*(elem.dldn(0, 0) + 2*elem.dldn(2, 0)) - elem.edgeLength(0))/(elem.edgeLength(0)*elem.edgeLength(1)),
//            -4/elem.edgeLength(1),
//            4/elem.edgeLength(1),
//            4/elem.edgeLength(1),
//            elem.doubleArea()/(elem.edgeLength(0)*elem.edgeLength(1)),
//            0,
//            -elem.doubleArea()/(elem.edgeLength(1)*elem.edgeLength(2));

//        std::cout << "pgc e1: " << (nodes * fde1).transpose() << "\n";
//        std::cout << "should be: " << pgConstraint[e1].transpose() << "\n";
//        std::cout << "pgc e2: " << (nodes * fde2).transpose() << "\n";
//        std::cout << "should be: " << pgConstraint[e2].transpose() << "\n";
//    }

    m_solved = true;
}


}  // namespace Vitelotte
