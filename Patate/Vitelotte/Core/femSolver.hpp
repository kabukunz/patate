/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "femSolver.h"


namespace Vitelotte
{

namespace internal
{

template <typename Solver>
struct CheckEigenSolverError
{
    static bool check(const Solver& solver, SolverError& error)
    {
        switch(solver.info()) {
        case Eigen::Success:        return false;
        case Eigen::NumericalIssue: error.error("Numerical issue"); return true;
        case Eigen::NoConvergence:  error.error("No convergence");  return true;
        case Eigen::InvalidInput:   error.error("Invalid matrix");  return true;
        }
        error.error("Unknown Eigen error");  return true;
        return true;
    }
};

//template <typename Matrix>
//bool CheckEigenSolverError<Eigen::SparseLU<Matrix> >::check(
//        const Eigen::SparseLU<Matrix>& solver, SolverError& error)
//{
//    if(solver.info() != Eigen::Success) {
//        error.error(solver.lastErrorMessage());
//        return true;
//    }
//    return false;
//}


template <typename Solver>
bool checkEigenSolverError(const Solver& solver, SolverError& error)
{
    return CheckEigenSolverError<Solver>::check(solver, error);
}


}


template < class _Mesh, class _ElementBuilder >
FemSolver<_Mesh, _ElementBuilder>::FemSolver(Mesh* _mesh, const ElementBuilder& elementBuilder)
  : m_mesh(_mesh),
    m_elementBuilder(elementBuilder),
    m_solved(false)
{
}


template < class _Mesh, class _ElementBuilder >
FemSolver<_Mesh, _ElementBuilder>::~FemSolver()
{
    resizeSpdBlocks(0);
    resizeSymBlocks(0);
}


template < class _Mesh, class _ElementBuilder >
void
FemSolver<_Mesh, _ElementBuilder>::build()
{
    unsigned nCoefficients = 0;
    m_error.resetStatus();
    m_solved = false;
    for(FaceIterator elem = m_mesh->facesBegin();
        elem != m_mesh->facesEnd(); ++elem)
    {
        nCoefficients += m_elementBuilder.nCoefficients(*m_mesh, *elem, &m_error);
    }

    TripletVector coefficients(nCoefficients);
    TripletVectorIterator it = coefficients.begin();
    for(FaceIterator elem = m_mesh->facesBegin();
        elem != m_mesh->facesEnd(); ++elem)
    {
        m_elementBuilder.addCoefficients(it, *m_mesh, *elem, &m_error);

        if(m_error.status() == SolverError::STATUS_ERROR)
        {
            return;
        }
    }
    assert(it == coefficients.end());

    m_stiffnessMatrix.resize(m_mesh->nNodes(), m_mesh->nNodes());
    m_stiffnessMatrix.setFromTriplets(
                coefficients.begin(), coefficients.end());

    m_type = m_elementBuilder.matrixType(*m_mesh);
//    std::cout << ((m_type == ElementBuilder::MATRIX_SPD)?
//                      "SPD\n": "Symetric\n");

    m_b.resize(m_mesh->nNodes(), m_mesh->nCoeffs());
    m_elementBuilder.setRhs(*m_mesh, m_b, &m_error);
    if(m_error.status() == SolverError::STATUS_ERROR)
    {
        return;
    }
}


template < class _Mesh, class _ElementBuilder >
void
FemSolver<_Mesh, _ElementBuilder>::sort()
{
    if(m_error.status() == SolverError::STATUS_ERROR)
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
void FemSolver<_Mesh, _ElementBuilder>::factorize()
{
    m_solved = false;

    if(m_error.status() == SolverError::STATUS_ERROR)
    {
        return;
    }

    // Apply permutation
    Eigen::PermutationMatrix<Eigen::Dynamic> perm(m_perm), permInv(perm.inverse());
    StiffnessMatrix mat;
    mat/*.template selfadjointView<Eigen::Lower>()*/ =
            m_stiffnessMatrix.template selfadjointView<Eigen::Lower>().twistedBy(permInv);

    unsigned nUnknowns = m_ranges.back();
    unsigned nConstraints = m_stiffnessMatrix.cols() - nUnknowns;

    m_consBlock = mat.topRightCorner(nUnknowns, nConstraints);

    resizeBlocks();

    unsigned nbRanges = m_ranges.size()-1;
//#ifdef _OPENMP
//#pragma omp parallel for schedule(static,1)
//#endif
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

//        std::cout << "range: " << start << " (" << m_perm[start] << "), "
//                  << size << ", nz=" << L.nonZeros() << " ("
//                  << 100. * L.nonZeros() / double(L.cols() * L.rows()) << "%)\n";

        switch(m_type)
        {
        case ElementBuilder::MATRIX_SPD:
            assert(k < m_spdBlocks.size());
            if(!m_spdBlocks[k]) m_spdBlocks[k] = new SPDFactorization;
            m_spdBlocks[k]->compute(L);
            if(internal::checkEigenSolverError(*m_spdBlocks[k], m_error)) return;
            break;
        case ElementBuilder::MATRIX_SYMETRIC:
            assert(k < m_symBlocks.size());
            if(!m_symBlocks[k]) m_symBlocks[k] = new SymFactorization;
            m_symBlocks[k]->analyzePattern(L);
            m_symBlocks[k]->factorize(L);
            if(internal::checkEigenSolverError(*m_symBlocks[k], m_error)) return;
            break;
        }
    }
}


template < class _Mesh, class _ElementBuilder >
void
FemSolver<_Mesh, _ElementBuilder>::solve()
{
    m_solved = false;

    if(m_error.status() == SolverError::STATUS_ERROR)
    {
        return;
    }

    // compute RHS
    unsigned nUnknowns = m_ranges.back();
    unsigned nConstraints = m_stiffnessMatrix.cols() - nUnknowns;
    Matrix constraints(nConstraints, m_mesh->nCoeffs());

    // As sort() keep constraints order, they will have the right indices.
    unsigned count = 0;
    for(unsigned i = 0; i < m_mesh->nNodes(); ++i)
    {
        if(m_mesh->isConstraint(Node(i)))
        {
            constraints.row(count++) =
                    m_mesh->value(Node(i)).template cast<Scalar>();
        }
    }
    assert(count == nConstraints);

    m_x.resize(nUnknowns, m_mesh->nCoeffs());
    for(unsigned i = 0; i < m_x.rows(); ++i)
    {
        m_x.row(i) = m_b.row(m_perm(i));
    }

    m_x -= m_consBlock * constraints;

    unsigned nbRanges = m_ranges.size()-1;
    for(unsigned k = 0; k < nbRanges; ++k)
    {
        int start = m_ranges[k];
        int size  = m_ranges[k+1]-start;

        switch(m_type)
        {
        case ElementBuilder::MATRIX_SPD:
            if(m_spdBlocks[k])
            {
                m_x.middleRows(start,size) = m_spdBlocks[k]->solve(m_x.middleRows(start, size));
                if(internal::checkEigenSolverError(*m_spdBlocks[k], m_error)) return;
            }
            break;
        case ElementBuilder::MATRIX_SYMETRIC:
            if(m_symBlocks[k])
            {
                m_x.middleRows(start,size) = m_symBlocks[k]->solve(m_x.middleRows(start, size));
                if(internal::checkEigenSolverError(*m_symBlocks[k], m_error)) return;
            }
            break;
        }
    }

    for(unsigned i = 0; i < nUnknowns; ++i)
    {
        assert(!m_mesh->isConstraint(Node(m_perm[i])));
        m_mesh->value(Node(m_perm[i])) = m_x.row(i).
                    template cast<typename Mesh::Scalar>();
    }

    m_solved = true;
}


template < class _Mesh, class _ElementBuilder >
void
FemSolver<_Mesh, _ElementBuilder>::resizeBlocks()
{
    unsigned nbRanges = m_ranges.size()-1;

    switch(m_type)
    {
    case ElementBuilder::MATRIX_SPD:
        resizeSpdBlocks(nbRanges);
        resizeSymBlocks(0);
        break;
    case ElementBuilder::MATRIX_SYMETRIC:
        resizeSpdBlocks(0);
        resizeSymBlocks(nbRanges);
        break;
    }
}


template < class _Mesh, class _ElementBuilder >
void
FemSolver<_Mesh, _ElementBuilder>::resizeSpdBlocks(unsigned size)
{
    for(unsigned i = size; i < m_spdBlocks.size(); ++i)
        delete m_spdBlocks[i];
    m_spdBlocks.resize(size, 0);
}


template < class _Mesh, class _ElementBuilder >
void
FemSolver<_Mesh, _ElementBuilder>::resizeSymBlocks(unsigned size)
{
    for(unsigned i = size; i < m_symBlocks.size(); ++i)
        delete m_symBlocks[i];
    m_symBlocks.resize(size, 0);
}


}  // namespace Vitelotte
