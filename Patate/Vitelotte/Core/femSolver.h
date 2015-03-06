/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _VITELOTTE_FEM_SOLVER_
#define _VITELOTTE_FEM_SOLVER_

#include <cassert>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "solverError.h"
#include "femUtils.h"
#include "vgMesh.h"


namespace Vitelotte
{


template < class _Mesh, class _ElementBuilder >
class FemSolver
{
public:
    typedef _Mesh Mesh;
    typedef _ElementBuilder ElementBuilder;

    typedef typename ElementBuilder::Scalar Scalar;
    typedef typename ElementBuilder::MatrixType MatrixType;

    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
    typedef Eigen::Triplet<Scalar> Triplet;
    typedef Eigen::SparseMatrix<Scalar> StiffnessMatrix;


    typedef std::vector<Triplet> TripletVector;
    typedef typename TripletVector::iterator TripletVectorIterator;

    typedef typename Mesh::Node Node;
    typedef typename Mesh::Vertex Vertex;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::FaceIterator FaceIterator;

public:
    inline FemSolver(Mesh* _inMesh,
                     const ElementBuilder& elementBuilder = ElementBuilder());
    ~FemSolver();

    void build();
    void sort();
    void factorize();
    void solve();

    inline bool isSolved() const { return m_solved; }
    inline const SolverError error() { return m_error; }

protected:
    typedef std::vector<unsigned> RangeVector;

    typedef Eigen::SimplicialLDLT<StiffnessMatrix> SPDFactorization;
    typedef Eigen::SparseLU<StiffnessMatrix> SymFactorization;
//    typedef Eigen::SparseQR<StiffnessMatrix, Eigen::COLAMDOrdering<int> > SymFactorization;

    typedef std::vector<SPDFactorization*> SPDBlocks;
    typedef std::vector<SymFactorization*> SymBlocks;

protected:
    void resizeBlocks();
    void resizeSpdBlocks(unsigned size);
    void resizeSymBlocks(unsigned size);

protected:
    Mesh* m_mesh;
    ElementBuilder m_elementBuilder;

    SolverError m_error;
    bool m_solved;

    // Build: Ax = b
    StiffnessMatrix m_stiffnessMatrix;
    Matrix m_b;
    MatrixType m_type;

    // Sort: premutation + ranges
    Eigen::VectorXi m_perm;
    RangeVector m_ranges;

    // Factorize: block-wise factorizations + constraint matrix
    StiffnessMatrix m_consBlock;
    SPDBlocks m_spdBlocks;
    SymBlocks m_symBlocks;

    // Solve
    Matrix m_x;


//    unsigned m_nbCells;
};


} // namespace Vitelotte

#include "femSolver.hpp"


#endif
