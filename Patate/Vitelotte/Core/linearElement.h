/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _VITELOTTE_LINEAR_ELEMENT_
#define _VITELOTTE_LINEAR_ELEMENT_


#include <Eigen/Core>

#include "../../common/defines.h"


namespace Vitelotte
{


template < typename _Scalar >
class LinearElement
{
public:
    typedef _Scalar Scalar;

    typedef Eigen::Matrix<Scalar, 2, 1> Vector;
    typedef Eigen::Matrix<Scalar, 3, 1> Values;
    typedef Eigen::Matrix<Scalar, 3, 2> Jacobian;
    typedef Eigen::Matrix<Scalar, 2, 2> Hessian;

    typedef Eigen::Matrix<Scalar, 3, 1> BarycentricCoord;

protected:
    typedef Eigen::Matrix<Scalar, 2, 2> Matrix2;
    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

    typedef Eigen::Matrix<Scalar, 3, 2> Matrix3x2;
    typedef Eigen::Matrix<Scalar, 2, 3> Matrix2x3;

public:
    MULTIARCH inline LinearElement(const Vector* pts)
    {
        for(unsigned i = 0; i < 3; ++i)
            m_points.col(i) = pts[i];
        computeFromPoints();
    }

    MULTIARCH inline LinearElement(
            const Vector& p0, const Vector& p1, const Vector& p2)
    {
        m_points.col(0) = p0;
        m_points.col(1) = p1;
        m_points.col(2) = p2;

        computeFromPoints();
    }

    MULTIARCH inline Vector point(unsigned pi, unsigned offset=0) const
    {
        assert(pi < 3 && offset < 3);
        return m_points.col((pi + offset) % 3);
    }

    MULTIARCH inline Scalar doubleArea() const { return m_2delta; }

    MULTIARCH inline BarycentricCoord barycentricCoordinates(
            const Vector& p) const
    {
        return m_lbf * (BarycentricCoord() << p, 1).finished();
    }

    MULTIARCH inline Values eval(const BarycentricCoord& bc) const
    {
        return bc;
    }

    MULTIARCH inline Scalar eval(unsigned bi, const BarycentricCoord& bc) const
    {
        assert(bi < 3);
        return bc(bi);
    }

    MULTIARCH inline const Eigen::Block<const Matrix3, 3, 2> jacobian(
            const BarycentricCoord& /*bc*/ = BarycentricCoord()) const
    {
        return m_lbf.template block<3, 2>(0, 0);
    }

    MULTIARCH inline const Vector gradient(
            unsigned bi,
            const BarycentricCoord& /*bc*/ = BarycentricCoord()) const
    {
        assert(bi < 3);
        return m_lbf.template block<1, 2>(bi, 0);
    }

protected:
    MULTIARCH inline void computeFromPoints()
    {
        m_2delta = (point(1).x() - point(0).x()) * (point(2).y() - point(1).y())
                 - (point(1).y() - point(0).y()) * (point(2).x() - point(1).x());

        for(int li = 0; li < 3; ++li)
        {
            m_lbf(li, 0) = point(li, 1).y() - point(li, 2).y();
            m_lbf(li, 1) = point(li, 2).x() - point(li, 1).x();
            m_lbf(li, 2) = point(li, 1).x() * point(li, 2).y()
                         - point(li, 2).x() * point(li, 1).y();
        }
        m_lbf /= m_2delta;
    }

protected:
    Matrix2x3 m_points;

    Scalar m_2delta;
    Matrix3 m_lbf;
};


} // namespace Vitelotte


#endif
