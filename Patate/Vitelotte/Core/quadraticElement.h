/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _VITELOTTE_QUADRATIC_ELEMENT_
#define _VITELOTTE_QUADRATIC_ELEMENT_


#include <Eigen/Core>

#include "../../common/defines.h"

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
    typedef Eigen::Matrix<Scalar, 6, 1> Values;
    typedef Eigen::Matrix<Scalar, 6, 2> Jacobian;
    typedef Eigen::Matrix<Scalar, 2, 2> Hessian;

    typedef Eigen::Matrix<Scalar, 3, 1> BarycentricCoord;

public:
    MULTIARCH inline QuadraticElement(const Vector* pts)
        : Base(pts)
    {}

    MULTIARCH inline QuadraticElement(
            const Vector& p0, const Vector& p1, const Vector& p2)
        : Base(p0, p1, p2)
    {}

    MULTIARCH inline Vector point(unsigned pi, unsigned offset=0) const
    {
        assert(pi < 3 && offset < 3);
        return m_points.col((pi + offset) % 3);
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
            return bc(bi) * (2 * bc(bi) - 1);
        return 4 * bc((bi + 1) % 3) * bc((bi + 2) % 3);
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
