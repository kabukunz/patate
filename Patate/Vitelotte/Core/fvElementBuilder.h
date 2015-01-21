/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _VITELOTTE_FV_ELEMENT_BUILDER_
#define _VITELOTTE_FV_ELEMENT_BUILDER_


#include <Eigen/Core>
#include <Eigen/Sparse>

#include "elementBuilderBase.h"

namespace Vitelotte
{


/**
 * Fraeijs de Veubeke's elements.
 *
 * FV elements have 6 value nodes, 3 at each vertex (v) and 3 at edge midpoints
 * (e), plus 3 gradient nodes corresponding to the mean value of the first
 * (outward) normal derivative along each edge (g). There are the basis
 * functions:
 *
 * \f{eqnarray*}{
     v_0 &=& \lambda_{0} \left(\lambda_{0} - \frac{1}{2}\right) \left(\lambda_{0} + 1\right)
             + 3 \lambda_{0} \lambda_{1} \lambda_{2}
             + \left( \frac{d \lambda_{0}}{d \boldsymbol{n}_{1}} + \frac{1}{2} \frac{d \lambda_{1}}{d \boldsymbol{n}_{1}} \right) g_1
             + \left( \frac{d \lambda_{0}}{d \boldsymbol{n}_{2}} + \frac{1}{2} \frac{d \lambda_{2}}{d \boldsymbol{n}_{2}} \right) g_2 \\

     e_0 &=& 4 \lambda_{0} \left(- 2 \lambda_{0} + 1\right) \left(- \lambda_{0} + 1\right) + 4 \lambda_{1} \lambda_{2} - 12 \lambda_{0} \lambda_{1} \lambda_{2}\\
     g_0 &=& - \frac{2 \Delta}{l_{0}} \lambda_{0} \left(\lambda_{0} - 1\right) \left(2 \lambda_{0} - 1\right)\\
   \f}
 *
 * where \f$\lambda_i\f$ is the barycentric coordinate for the vertex \f$i\f$,
 * \f$\Delta\f$ is the area of the triangle, \f$l_i\f$ is the length of the
 * edge \f$i\f$ opposed to the vertex \f$i\f$ and \f$\boldsymbol{n}_i\f$ is the
 * (clockwise, normalized) normal of the edge \f$i\f$. The others basis
 * functions can be obtained by cyclic permutation of the indices.
 */
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
