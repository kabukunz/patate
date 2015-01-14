#ifndef _VITELOTTE_FV_ELEMENT_BUILDER_
#define _VITELOTTE_FV_ELEMENT_BUILDER_


#include <Eigen/Core>
#include <Eigen/Sparse>

#include "elementBuilderBase.h"

namespace Vitelotte
{


template < class _Mesh, typename _Scalar = typename _Mesh::Scalar >
class FVElementBuilder : public ElementBuilderBase
{
public:
    typedef _Scalar Scalar;
    typedef _Mesh Mesh;

    typedef Eigen::Matrix<Scalar, Mesh::Dim, 1> Vector;
    typedef Eigen::Triplet<Scalar> Triplet;

protected:
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<Scalar, 6, 1> Vector6;
    typedef typename Mesh::Face Face;

public:
    inline FVElementBuilder(Scalar sigma = Scalar(.5));

    unsigned nCoefficients(const Mesh& mesh, Face element) const;

    template < typename InIt >
    void addCoefficients(InIt& it, const Mesh& mesh, Face element);

protected:
    template < typename InIt >
    void processFV1Element(InIt& it, const Mesh& mesh, Face element);

    template < typename InIt >
    void processFV1ElementFlat(InIt& it, const Mesh& mesh, Face element) const;


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

    inline Vector3 funcVertexBasis(
            const int _i1, int _deriv, const Vector3& _a, const Vector3& _b,
            const Vector3& _c, const Vector3& _d, const Vector3& _l,
            const Scalar _area) const;
    inline Vector3 funcMidpointBasis(
            const int _i1, int _deriv, const Vector3& _a, const Vector3& _b,
            const Vector3& _c, const Vector3& _d, const Vector3& _l,
            const Scalar _area) const;
    inline Vector3 funcMidpointDerivBasis(
            const int _i1, int _deriv, const Vector3& _a, const Vector3& _b,
            const Vector3& _c, const Vector3& _d, const Vector3& _l,
            const Scalar _area) const;

    inline Scalar integrateQuadTriangle(
            const Vector* _v, const Vector6& _f, Scalar _area) const;

    inline Vector6 multBasis(const Vector3& _a, const Vector3& _b) const;

private:
    Scalar m_sigma;
};


} // namespace Vitelotte

#include "fvElementBuilder.hpp"


#endif
