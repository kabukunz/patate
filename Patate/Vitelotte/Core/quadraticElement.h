/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _VITELOTTE_QUADRATIC_ELEMENT_
#define _VITELOTTE_QUADRATIC_ELEMENT_


#include <Eigen/Core>

#include "linearElement.h"


namespace Vitelotte
{


template < typename _Scalar >
class QuadraticElement : protected LinearElement<_Scalar>
{
public:
    typedef _Scalar Scalar;
    typedef LinearElement<Scalar> Base;

    typedef Eigen::Matrix<Scalar, 2, 1> Vector;
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<Scalar, 6, 1> Vector6;

//    typedef Eigen::Matrix<Scalar, 2, 2> Matrix2;
//    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

    typedef Eigen::Matrix<Scalar, 6, 2> Matrix6x2;

public:
    inline QuadraticElement(const Vector* pts)
        : Base(pts)
    {}

    inline QuadraticElement(const Vector& p0, const Vector& p1, const Vector& p2)
        : Base(p0, p1, p2)
    {}

    inline Vector point(unsigned pi, unsigned offset=0) const
    {
        assert(pi < 3 && offset < 3);
        return m_points.col((pi + offset) % 3);
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
            return bc(bi) * (2 * bc(bi) - 1);
        return 4 * bc((bi + 1) % 3) * bc((bi + 2) % 3);
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
            return Base::gradient(bi, bc) * (4 * bc(bi) - 1);

        unsigned i1 = (bi + 1) % 3;
        unsigned i2 = (bi + 2) % 3;
        return 4 * (Base::gradient(i1, bc) * bc(i2) + Base::gradient(i2, bc) * bc(i1));
    }


protected:
    using Base::m_points;

    using Base::m_2delta;
    using Base::m_lbf;
};


} // namespace Vitelotte


#endif
