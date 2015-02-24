/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _VITELOTTE_MORLEY_ELEMENT_
#define _VITELOTTE_MORLEY_ELEMENT_


#include <Eigen/Core>

#include "../../common/defines.h"

#include "linearElement.h"


namespace Vitelotte
{


template < typename _Scalar >
class MorleyElement : protected LinearElement<_Scalar>
{
public:
    typedef _Scalar Scalar;
    typedef LinearElement<Scalar> Base;

    typedef Eigen::Matrix<Scalar, 2, 1> Vector;
    typedef Eigen::Matrix<Scalar, 6, 1> Values;
    typedef Eigen::Matrix<Scalar, 6, 2> Jacobian;
    typedef Eigen::Matrix<Scalar, 2, 2> Hessian;

    typedef Eigen::Matrix<Scalar, 3, 1> BarycentricCoord;

protected:
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;

    typedef Eigen::Matrix<Scalar, 2, 3> Matrix2x3;
    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

public:
    MULTIARCH inline MorleyElement(const Vector* pts)
        : Base(pts)
    {
        computeFromPoints();
    }

    MULTIARCH inline MorleyElement(
            const Vector& p0, const Vector& p1, const Vector& p2)
        : Base(p0, p1, p2)
    {
        computeFromPoints();
    }

    using Base::point;
    using Base::doubleArea;

    using Base::barycentricCoordinates;

    MULTIARCH inline Values eval(const BarycentricCoord& bc) const
    {
        Values basis;
        for(unsigned i = 0; i < 6; ++i)
            basis(i) = eval(i, bc);
        return basis;
    }

    MULTIARCH inline Scalar eval(unsigned bi, const BarycentricCoord& bc) const
    {
        assert(bi < 6);
        if(bi < 3)
        {
            unsigned bi1 = (bi + 1) % 3;
            unsigned bi2 = (bi + 2) % 3;
            return bc(bi)*bc(bi)
                 + m_dldn(bi, bi1) * eval(bi1 + 3, bc)
                 + m_dldn(bi, bi2) * eval(bi2 + 3, bc);
        }
        bi -= 3;
        return bc(bi) * (bc(bi) - 1) / m_dldn(bi, bi);
    }

    MULTIARCH inline const Jacobian jacobian(const BarycentricCoord& bc) const
    {
        Jacobian j;
        for(unsigned i = 0; i < 6; ++i)
            j.row(i) = gradient(i, bc);
        return j;
    }

    MULTIARCH inline const Vector gradient(unsigned bi,
                                           const BarycentricCoord& bc) const
    {
        assert(bi < 6);
        if(bi < 3)
        {
            unsigned bi1 = (bi + 1) % 3;
            unsigned bi2 = (bi + 2) % 3;
            return Base::gradient(bi, bc) * (2 * bc(bi))
                 + m_dldn(bi, bi1) * gradient(bi1 + 3, bc)
                 + m_dldn(bi, bi2) * gradient(bi2 + 3, bc);
        }
        bi -= 3;
        return Base::gradient(bi, bc) * ((2 * bc(bi) - 1) / m_dldn(bi, bi));
    }

    MULTIARCH inline void hessian(const BarycentricCoord& bc, Hessian* h) const
    {
        for(unsigned i = 0; i < 6; ++i)
            h[i] = hessian(i, bc);
    }

    MULTIARCH inline Hessian hessian(unsigned bi,
                                     const BarycentricCoord& bc) const
    {
        Hessian h;
        for(int d0 = 0; d0 < 2; ++d0)
        {
            for(int d1 = d0; d1 < 2; ++d1)
            {
                h(d0, d1) = 2.
                          * Base::jacobian()(bi%3, d0)
                          * Base::jacobian()(bi%3, d1);
            }
        }
        h(1, 0) = h(0, 1);

        if(bi < 3)
        {
            unsigned bi1 = (bi + 1) % 3;
            unsigned bi2 = (bi + 2) % 3;
            return h
                 + m_dldn(bi, bi1) * hessian(bi1 + 3, bc)
                 + m_dldn(bi, bi2) * hessian(bi2 + 3, bc);
        }

        bi -= 3;
        return h / m_dldn(bi, bi);
    }

protected:
    MULTIARCH inline void computeFromPoints()
    {
        Matrix2x3 vs;
        for(int i = 0; i < 3; ++i)
            vs.col(i) = point(i, 2) - point(i, 1);

        for(int i = 0; i < 3; ++i)
            m_eLen(i) = vs.col(i).norm();

        for(int ni = 0; ni < 3; ++ni)
            for(int li = 0; li < 3; ++li)
                m_dldn(li, ni) =
                        vs.col(li).dot(vs.col(ni)) / (m_2delta * m_eLen(ni));
    }


protected:
    using Base::m_points;

    using Base::m_2delta;
    using Base::m_lbf;

    Vector3 m_eLen;
    Matrix3 m_dldn;
};


} // namespace Vitelotte


#endif
