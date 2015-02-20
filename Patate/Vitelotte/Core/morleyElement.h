/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _VITELOTTE_MORLEY_ELEMENT_
#define _VITELOTTE_MORLEY_ELEMENT_


#include <Eigen/Core>

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
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<Scalar, 6, 1> Vector6;

    typedef Eigen::Matrix<Scalar, 2, 3> Matrix2x3;
    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

    typedef Eigen::Matrix<Scalar, 6, 2> Matrix6x2;

public:
    inline MorleyElement(const Vector* pts)
        : Base(pts)
    {
        computeFromPoints();
    }

    inline MorleyElement(const Vector& p0, const Vector& p1, const Vector& p2)
        : Base(p0, p1, p2)
    {
        computeFromPoints();
    }

    using Base::point;
    using Base::doubleArea;

    using Base::barycentricCoordinates;

    inline Vector6 eval(const Vector& p) const
    {
        return eval(barycentricCoordinates(p));
    }

    inline Vector6 eval(const Vector3& bc) const
    {
        Vector6 basis;
        for(unsigned i = 0; i < 6; ++i)
            basis(i) = eval(i, bc);
        return basis;
    }

    inline Scalar eval(unsigned bi, const Vector& p) const
    {
        assert(bi < 6);
        return eval(bi, barycentricCoordinates(p));
    }

    inline Scalar eval(unsigned bi, const Vector3& bc) const
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

    inline const Matrix6x2 jacobian(const Vector& p) const
    {
        return jacobian(barycentricCoordinates(p));
    }

    inline const Matrix6x2 jacobian(const Vector3& bc) const
    {
        Matrix6x2 j;
        for(unsigned i = 0; i < 6; ++i)
            j.row(i) = gradient(i, bc);
        return bc;
    }

    inline const Vector gradient(unsigned bi, const Vector& p) const
    {
        assert(bi < 6);
        return gradient(bi, barycentricCoordinates(p));
    }

    inline const Vector gradient(unsigned bi, const Vector3& bc) const
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


protected:
    void computeFromPoints()
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
