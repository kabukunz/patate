#ifndef _FEM_UTILS_H_
#define _FEM_UTILS_H_

#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <Eigen/StdVector>
#include <vector>

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
inline T lerp(FemScalar _v, const T& _p0, const T& _p1)
{
    return (1. - _v) * _p0 + _v * _p1;
}

template <class Derived0, class Derived1>
inline typename Eigen::MatrixBase<Derived0>::Scalar det2(const Eigen::MatrixBase<Derived0>& _v0, const Eigen::MatrixBase<Derived1>& _v1)
{
    return _v0.x() * _v1.y() - _v0.y() * _v1.x();
}

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
inline typename Eigen::MatrixBase<Derived0>::Scalar angleCcw(const Eigen::MatrixBase<Derived0>& _v0, const Eigen::MatrixBase<Derived1>& _v1)
{
	typedef typename Eigen::MatrixBase<Derived0>::Scalar Scalar;

	Scalar alpha = std::min(Scalar(M_PI), std::max(Scalar(0.), std::acos(std::max(Scalar(-1.), std::min(Scalar(1.), _v0.dot(_v1))))));
	return (det2(_v0, _v1) > 0) ? alpha : (M_PI * 2 - alpha);
}

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
                           const FemScalar _area)
{
    const size_t i2 = (_i1+1)%3;
    const size_t i3 = (_i1+2)%3;

    const FemVector3 d1 = (_deriv&Deriv_0_Y)? _c: _b;
    const FemVector3 d2 = (_deriv&Deriv_1_Y)? _c: _b;

    FemScalar factorA = d1(_i1)*d2(_i1)/(4.*_area*_area*_area);
    FemScalar factorB = 3./(8.*_area*_area*_area);

    FemScalar factorC = -3.*_d(i2)*d1(i2)*d2(i2)/(2.*_area*_area*_area);
    FemScalar factorD = 3.*_d(i3)*d1(i3)*d2(i3)/(2.*_area*_area*_area);

    FemVector3 comb(d1(i2)*d2(i3) + d1(i3)*d2(i2),
                    d1(_i1)*d2(i3) + d1(i3)*d2(_i1),
                    d1(_i1)*d2(i2) + d1(i2)*d2(_i1));

    return factorA * FemVector3(3.*_a(_i1) + _area, 3.*_b(_i1), 3.*_c(_i1))
            +  factorB * FemVector3(
            (_a(_i1)*comb(0) + _a(i2)*comb(1) + _a(i3)*comb(2)),
            (_b(_i1)*comb(0) + _b(i2)*comb(1) + _b(i3)*comb(2)),
            (_c(_i1)*comb(0) + _c(i2)*comb(1) + _c(i3)*comb(2)))
            +  factorC * FemVector3(_a(i2) - _area, _b(i2), _c(i2))
            +  factorD * FemVector3(_a(i3) - _area, _b(i3), _c(i3));
}

inline FemVector3 funcMidpointBasis(const size_t _i1, int _deriv, const FemVector3& _a, const FemVector3& _b,
                             const FemVector3& _c, const FemVector3& _d, const FemVector3& _l,
                             const FemScalar _area)
{
    const size_t i2 = (_i1+1)%3;
    const size_t i3 = (_i1+2)%3;

    const FemVector3 d1 = (_deriv&Deriv_0_Y)? _c: _b;
    const FemVector3 d2 = (_deriv&Deriv_1_Y)? _c: _b;

    FemScalar factorA = 6.*d1(_i1)*d2(_i1)/(_area*_area*_area);
    FemScalar factorB = -3./(2.*_area*_area*_area);

    FemVector3 comb(d1(i2)*d2(i3) + d1(i3)*d2(i2),
                    d1(_i1)*d2(i3) + d1(i3)*d2(_i1),
                    d1(_i1)*d2(i2) + d1(i2)*d2(_i1));

    return FemVector3(comb(0) / (_area*_area), 0., 0.)
            +  factorA * FemVector3(_a(_i1) - _area, _b(_i1), _c(_i1))
            +  factorB * FemVector3(
            (_a(_i1)*comb(0) + _a(i2)*comb(1) + _a(i3)*comb(2)),
            (_b(_i1)*comb(0) + _b(i2)*comb(1) + _b(i3)*comb(2)),
            (_c(_i1)*comb(0) + _c(i2)*comb(1) + _c(i3)*comb(2)));
}

inline FemVector3 funcMidpointDerivBasis(const size_t _i1, int _deriv, const FemVector3& _a, const FemVector3& _b,
                                  const FemVector3& _c, const FemVector3& _d, const FemVector3& _l,
                                  const FemScalar _area)
{
    const FemVector3& d1 = (_deriv & Deriv_0_Y) ? _c : _b;
    const FemVector3& d2 = (_deriv & Deriv_1_Y) ? _c : _b;

    FemScalar factorA = -3. * d1(_i1) * d2(_i1) / (_l(_i1) * _area * _area);

    return factorA * FemVector3(_a(_i1) - _area, _b(_i1), _c(_i1));
}

inline FemScalar integrateQuadTriangle(const FemVector* _v, const FemVector6& _f, FemScalar _area)
{
    return +(
                +_f(0)
                +(
                    +_f(1)*_v[2](0)
                    +_f(2)*_v[2](1)
                ) / 3.
                +(
                    -_f(1)*_v[1](0)
                    -_f(2)*_v[1](1)
                ) / 3.
                +(
                    +_f(3)*_v[2](0)*_v[2](0)
                    +_f(4)*_v[2](0)*_v[2](1)
                    +_f(5)*_v[2](1)*_v[2](1)
                ) / 6.
                +(
                    -2.*_f(3)*_v[2](0)*_v[1](0)
                    -_f(4)*_v[2](0)*_v[1](1)
                    -_f(4)*_v[2](1)*_v[1](0)
                    -2.*_f(5)*_v[2](1)*_v[1](1)
                ) / 12.
                +(
                    +_f(3)*_v[1](0)*_v[1](0)
                    +_f(4)*_v[1](0)*_v[1](1)
                    +_f(5)*_v[1](1)*_v[1](1)
                ) / 6.
        ) * _area;
}

inline FemVector6 multBasis(const FemVector3& _a, const FemVector3& _b)
{
    return  (
                FemVector6() << _a(0)*_b(0), _a(0)*_b(1) + _a(1)*_b(0), _a(0)*_b(2) + _a(2)*_b(0),
                                _a(1)*_b(1), _a(1)*_b(2) + _a(2)*_b(1), _a(2)*_b(2)
            ).finished();
}

#endif