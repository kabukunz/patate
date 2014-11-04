#ifndef _FEM_SOLVER_H_
#define _FEM_SOLVER_H_

#include <cassert>

#include <Eigen/Core>
#include <Eigen/Sparse>

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

    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
    typedef Eigen::Triplet<Scalar> Triplet;
    typedef Eigen::SparseMatrix<Scalar> StiffnessMatrix;


    typedef std::vector<Triplet> TripletVector;
    typedef typename TripletVector::iterator TripletVectorIterator;

    typedef typename Mesh::Node Node;
    typedef typename Mesh::Vertex Vertex;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::FaceIterator FaceIterator;

    typedef std::vector<unsigned> RangeVector;

public:
    inline FemSolver(Mesh* _inMesh,
                     const ElementBuilder& elementBuilder = ElementBuilder());

    inline void build();
    inline void sort();
    inline void solve();

    inline bool isSolved() const { return m_solved; }

protected:
    template<typename SpMatrix, typename Rhs, typename Res>
    inline void multiSolve(const SpMatrix& _A, const Rhs& _b, Res& _x, unsigned& _nbRanges);

    template<typename SpMatrix>
    inline void depthFirstOrdering(const SpMatrix& _mat, Eigen::VectorXi& _p, std::vector<int>& _ranges);

protected:
    Mesh* m_mesh;
    ElementBuilder m_elementBuilder;
    Eigen::VectorXi m_perm;
    RangeVector m_ranges;

    bool m_solved;

    StiffnessMatrix m_stiffnessMatrix;
    Matrix m_x;

    unsigned m_nbCells;
};


} // namespace Vitelotte

#include "femSolver.hpp"


#endif
