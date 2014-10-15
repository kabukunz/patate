#ifndef _FEM_SOLVER_H_
#define _FEM_SOLVER_H_

#include <iostream>
#include <cassert>

#include "femUtils.h"
#include "quadraticMesh.h"


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

    typedef typename Mesh::Vertex Vertex;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::FaceIterator FaceIterator;

    typedef std::vector<unsigned> RangeVector;

//public:
//    static FemMatrix33 m_linM1;
//    static FemMatrix33 m_linM2;
//    static FemMatrix33 m_linM3;

//    static FemMatrix66 m_quadM1;
//    static FemMatrix66 m_quadM2;
//    static FemMatrix66 m_quadM3;

//    static FemMatrix1010 m_cubM1;
//    static FemMatrix1010 m_cubM2;
//    static FemMatrix1010 m_cubM3;

//public:
//    static void initMatrices();

public:
    inline FemSolver(Mesh* _inMesh,
                     const ElementBuilder& elementBuilder = ElementBuilder());

    inline void build();
    inline void sort();
    inline void solve();

    inline bool isSolved() const { return m_solved; }

private:
//    inline void processElement(const Face& _elem, FemVector* _v, bool* _orient);
//    inline void processQuadraticElement(const Face& _elem, FemVector* _v, FemMatrixX& _elemStiffness);
//    inline void processFV1Element(const Face& _elem, FemVector* _v, bool* _orient, FemMatrixX& _elemStiffness);
//    inline void processFV1ElementFlat(const Face& _elem, FemVector* _v, bool* _orient, FemMatrixX& _elemStiffness);

//    inline void addToStiffness(Node _i, Node _j, FemScalar _value);
//    inline size_t rowFromNode(Node _node);

//    inline void updateResult();

    template<typename SpMatrix, typename Rhs, typename Res>
    inline void multiSolve(const SpMatrix& _A, const Rhs& _b, Res& _x, unsigned& _nbRanges);

    template<typename SpMatrix>
    inline void depthFirstOrdering(const SpMatrix& _mat, Eigen::VectorXi& _p, std::vector<int>& _ranges);

private:
    Mesh* m_mesh;
    ElementBuilder m_elementBuilder;
    Eigen::VectorXi m_perm;
    RangeVector m_ranges;

    bool m_solved;

    StiffnessMatrix m_stiffnessMatrix;
    FemMatrixX m_x;

    unsigned m_nbCells;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////Implementation//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//FemMatrix33 FemSolver::m_linM1;
//FemMatrix33 FemSolver::m_linM2;
//FemMatrix33 FemSolver::m_linM3;

//FemMatrix66 FemSolver::m_quadM1;
//FemMatrix66 FemSolver::m_quadM2;
//FemMatrix66 FemSolver::m_quadM3;

//FemMatrix1010 FemSolver::m_cubM1;
//FemMatrix1010 FemSolver::m_cubM2;
//FemMatrix1010 FemSolver::m_cubM3;


#include "femSolver.hpp"

} // namespace Vitelotte

#endif
