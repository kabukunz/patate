#ifndef _FEM_UTILS_H_
#define _FEM_UTILS_H_

#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <Eigen/StdVector>
#include <vector>


namespace Vitelotte
{


typedef double FemScalar;

typedef Eigen::Matrix<FemScalar, 2, 1> FemVector;
typedef Eigen::Matrix<FemScalar, 3, 1> FemVector3;
typedef Eigen::Matrix<FemScalar, 6, 1> FemVector6;
typedef Eigen::Matrix<FemScalar, 10, 1> FemVector10;
typedef Eigen::Matrix<FemScalar, Eigen::Dynamic, 1> FemVectorX;


typedef Eigen::Matrix<FemScalar, 4, 1> FemColor;

typedef Eigen::Matrix<FemScalar, 2, 2> FemMatrix;
typedef Eigen::Matrix<FemScalar, 3, 3> FemMatrix33;
typedef Eigen::Matrix<FemScalar, 6, 6> FemMatrix66;
typedef Eigen::Matrix<FemScalar, 10, 10> FemMatrix1010;
typedef Eigen::Matrix<FemScalar, Eigen::Dynamic, Eigen::Dynamic> FemMatrixX;

typedef Eigen::SparseMatrix<FemScalar> FemSMatrixX;



template<class T>
inline T lerp(FemScalar _v, const T& _p0, const T& _p1);

template <class Derived0, class Derived1>
inline typename Eigen::MatrixBase<Derived0>::Scalar det2(
        const Eigen::MatrixBase<Derived0>& _v0, const Eigen::MatrixBase<Derived1>& _v1);

/**
 * \brief Compute the counter-clockwise angle between `_v0` and `_v1`.
 *
 * Both vectors in input must be normalized. If `_v0` approximately equals `_v1`
 * the result may be close to 0 or 2*pi.
 *
 * @param _v0 The first normalized vector.
 * @param _v1 The second normalized vector.
 * @return The counter-clockwise angle between `_v0` and `_v1` comprised between
 * 0 and 2*pi.
 */
template <class Derived0, class Derived1>
inline typename Eigen::MatrixBase<Derived0>::Scalar angleCcw(
        const Eigen::MatrixBase<Derived0>& _v0, const Eigen::MatrixBase<Derived1>& _v1);

enum PartialDerivative
{
    Deriv_0_X = 0,
    Deriv_1_X = 0,
    Deriv_0_Y = 1 << 0,
    Deriv_1_Y = 1 << 1,
    Deriv_XX = Deriv_0_X | Deriv_1_X,
    Deriv_XY = Deriv_0_X | Deriv_1_Y,
    Deriv_YX = Deriv_0_Y | Deriv_1_X,
    Deriv_YY = Deriv_0_Y | Deriv_1_Y
};

inline FemVector3 funcVertexBasis(const size_t _i1, int _deriv, const FemVector3& _a, const FemVector3& _b,
                           const FemVector3& _c, const FemVector3& _d, const FemVector3& _l,
                           const FemScalar _area);
inline FemVector3 funcMidpointBasis(const size_t _i1, int _deriv, const FemVector3& _a, const FemVector3& _b,
                             const FemVector3& _c, const FemVector3& _d, const FemVector3& _l,
                             const FemScalar _area);
inline FemVector3 funcMidpointDerivBasis(const size_t _i1, int _deriv, const FemVector3& _a, const FemVector3& _b,
                                  const FemVector3& _c, const FemVector3& _d, const FemVector3& _l,
                                  const FemScalar _area);

inline FemScalar integrateQuadTriangle(const FemVector* _v, const FemVector6& _f, FemScalar _area);

inline FemVector6 multBasis(const FemVector3& _a, const FemVector3& _b);


#include "femUtils.hpp"

} // namespace Vitelotte

#endif
