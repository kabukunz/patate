#ifndef _FEM_SOLVER_H_
#define _FEM_SOLVER_H_

#include <iostream>
#include <cassert>

#include "femUtils.h"
#include "femInMesh.h"


namespace Vitelotte
{


class FemSolver
{
public:
    enum
    {
        nbChannels = FemColor::RowsAtCompileTime
    };

    typedef surface_mesh::Surface_mesh::Vertex Vertex;
    typedef surface_mesh::Surface_mesh::Vertex_around_face_circulator Vertex_around_face_circulator;
    typedef surface_mesh::Surface_mesh::Face Face;
    typedef surface_mesh::Surface_mesh::Face_iterator Face_iterator;

public:
    static FemMatrix33 m_linM1;
    static FemMatrix33 m_linM2;
    static FemMatrix33 m_linM3;

    static FemMatrix66 m_quadM1;
    static FemMatrix66 m_quadM2;
    static FemMatrix66 m_quadM3;

    static FemMatrix1010 m_cubM1;
    static FemMatrix1010 m_cubM2;
    static FemMatrix1010 m_cubM3;

public:
    static void initMatrices();

public:
    inline FemSolver(FemInMesh* _inMesh, FemScalar _sigma=0.5);

    inline void buildMatrices(bool _biharmonic);
    inline void solve();

    inline bool isSolved() const { return m_solved; }

private:
    inline void processElement(const Face& _elem, FemVector* _v, bool* _orient);
    inline void processQuadraticElement(const Face& _elem, FemVector* _v, FemMatrixX& _elemStiffness);
    inline void processFV1Element(const Face& _elem, FemVector* _v, bool* _orient, FemMatrixX& _elemStiffness);
    inline void processFV1ElementFlat(const Face& _elem, FemVector* _v, bool* _orient, FemMatrixX& _elemStiffness);

    inline void addToStiffness(Node _i, Node _j, FemScalar _value);
    inline size_t rowFromNode(Node _node);

    inline void updateResult();

    template<typename SpMatrix, typename Rhs, typename Res>
    inline void multiSolve(const SpMatrix& _A, const Rhs& _b, Res& _x, unsigned& _nbRanges);

    template<typename SpMatrix>
    inline void depthFirstOrdering(const SpMatrix& _mat, Eigen::VectorXi& _p, std::vector<int>& _ranges);

private:
    FemInMesh* m_inMesh;

    bool m_biharmonic;
    bool m_solved;
    FemScalar m_sigma;

    FemSMatrixX m_stiffnessMatrixTopLeft;
    FemSMatrixX m_stiffnessMatrixTopRight;
    FemMatrixX m_b;
    FemMatrixX m_rhs;
    FemMatrixX m_x;

    unsigned m_nbCells;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////Implementation//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

FemMatrix33 FemSolver::m_linM1;
FemMatrix33 FemSolver::m_linM2;
FemMatrix33 FemSolver::m_linM3;

FemMatrix66 FemSolver::m_quadM1;
FemMatrix66 FemSolver::m_quadM2;
FemMatrix66 FemSolver::m_quadM3;

FemMatrix1010 FemSolver::m_cubM1;
FemMatrix1010 FemSolver::m_cubM2;
FemMatrix1010 FemSolver::m_cubM3;


#include "femSolver.hpp"

} // namespace Vitelotte

#endif
