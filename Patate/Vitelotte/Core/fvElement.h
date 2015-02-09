/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _VITELOTTE_FV_ELEMENT_
#define _VITELOTTE_FV_ELEMENT_


#include <Eigen/Core>

#include "linearElement.h"


namespace Vitelotte
{


template < typename _Scalar >
class FVElement : protected LinearElement<_Scalar>
{
public:
    typedef _Scalar Scalar;
    typedef LinearElement<Scalar> Base;

    typedef Eigen::Matrix<Scalar, 2, 1> Vector;
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<Scalar, 9, 1> Vector9;

    typedef Eigen::Matrix<Scalar, 2, 2> Matrix2;
    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

    typedef Eigen::Matrix<Scalar, 2, 3> Matrix2x3;
    typedef Eigen::Matrix<Scalar, 9, 2> Vector9x2;


public:
    inline FVElement(const Vector* pts)
        : Base(pts)
    {
        computeFromPoints();
    }

    inline FVElement(const Vector& p0, const Vector& p1, const Vector& p2)
        : Base(p0, p1, p2)
    {
        computeFromPoints();
    }

    using Base::point;
    using Base::doubleArea;

    inline Scalar edgeLength(unsigned ei) const
    {
        assert(ei < 3);
        return m_eLen(ei);
    }

    inline const Matrix3& dldn() const
    {
        return m_dldn;
    }

    inline Scalar dldn(unsigned li, unsigned ni) const
    {
        assert(li < 3 && ni < 3);
        return m_dldn(li, ni);
    }

    using Base::barycentricCoordinates;

    inline Vector9 eval(const Vector& p) const
    {
        Vector3 bc = barycentricCoordinates(p);
        return eval(bc);
    }

    inline Vector9 eval(const Vector3& bc) const
    {
        Vector9 eb;
        Scalar bubble = _bubble(bc);

        for(int i = 0; i < 3; ++i)
        {
            Scalar gse = _gradientSubExpr(i, bc);
            eb(i + 3) = 4 * _edgeSubExpr(i, bc)
                      + 4 * gse
                      - 12 * bubble;
            eb(i + 6) = _gradientFactor(i) * gse;
        }

        for(int i = 0; i < 3; ++i)
        {
            unsigned i1 = (i+1) % 3;
            unsigned i2 = (i+2) % 3;
            eb(i) = _vertexSubExpr(i, bc)
                  + 3 * bubble
                  + _vertexGradientFactor(i, i1) * eb(i1 + 6)
                  + _vertexGradientFactor(i, i2) * eb(i2 + 6);
        }

        return eb;
    }

    Scalar eval(unsigned bi, const Vector& p) const
    {
        assert(bi < 9);
        Vector3 bc = barycentricCoordinates(p);
        return eval(bi, bc);
    }

    inline Scalar eval(unsigned bi, const Vector3& bc) const
    {
        assert(bi < 9);

        Scalar v;
        unsigned i = bi % 3;
        switch(bi/3)
        {
        case 0:
        {
            unsigned i1 = (i+1) % 3;
            unsigned i2 = (i+2) % 3;
            v = _vertexSubExpr(i, bc)
              + 3 * _bubble(bc)
              + _vertexGradientFactor(i, i1) * _gradientFactor(i1) * _gradientSubExpr(i1, bc)
              + _vertexGradientFactor(i, i2) * _gradientFactor(i2) * _gradientSubExpr(i2, bc);
            break;
        }
        case 1:
        {
            v =  4 * _edgeSubExpr(i, bc)
              +  4 * _gradientSubExpr(i, bc)
              - 12 * _bubble(bc);
            break;
        }
        case 2:
        {
            v = _gradientFactor(i) * _gradientSubExpr(i, bc);
            break;
        }
        }

        return v;
    }

    inline Vector9x2 jacobian(const Vector& p) const
    {
        Vector3 bc = barycentricCoordinates(p);
        return jacobian(bc);
    }

    inline Vector9x2 jacobian(const Vector3& bc) const
    {
        Vector9x2 grad;
        Vector bubbleGradient = _bubbleGradient(bc);

        for(int i = 0; i < 3; ++i)
        {
            Vector gseGradient = _gradientSubExprGradient(i, bc);
            grad.row(i + 3) = 4 * _edgeSubExprGradient(i, bc)
                            + 4 * gseGradient
                            - 12 * bubbleGradient;
            grad.row(i + 6) = _gradientFactor(i) * gseGradient;
        }

        for(int i = 0; i < 3; ++i)
        {
            unsigned i1 = (i+1) % 3;
            unsigned i2 = (i+2) % 3;
            Vector v1 = grad.row(i1 + 6);
            Vector v2 = grad.row(i2 + 6);
            grad.row(i) = _vertexSubExprGradient(i, bc)
                        + 3 * bubbleGradient
                        + _vertexGradientFactor(i, i1) * v1 //grad.row(i1 + 6)
                        + _vertexGradientFactor(i, i2) * v2; //grad.row(i2 + 6);
        }

        return grad;
    }

    inline Vector gradient(unsigned bi, const Vector& p) const
    {
        Vector3 bc = barycentricCoordinates(p);
        return gradient(bi, bc);
    }

    inline Vector gradient(unsigned bi, const Vector3& bc) const
    {
        assert(bi < 9);

        Vector grad;
        unsigned i = bi % 3;
        switch(bi/3)
        {
        case 0:
        {
            unsigned i1 = (i+1) % 3;
            unsigned i2 = (i+2) % 3;
            grad = _vertexSubExprGradient(i, bc)
                 + 3 * _bubbleGradient(bc)
                 + _vertexGradientFactor(i, i1) * _gradientFactor(i1) * _gradientSubExprGradient(i1, bc)
                 + _vertexGradientFactor(i, i2) * _gradientFactor(i2) * _gradientSubExprGradient(i2, bc);
            break;
        }
        case 1:
        {
            grad =  4 * _edgeSubExprGradient(i, bc)
                 +  4 * _gradientSubExprGradient(i, bc)
                 - 12 * _bubbleGradient(bc);
            break;
        }
        case 2:
        {
            grad = _gradientFactor(i) * _gradientSubExprGradient(i, bc);
            break;
        }
        }

        return grad;
    }

    inline void hessian(const Vector3& bc, Matrix2* h) const
    {
        Matrix2 bubbleHessian = _bubbleHessian(bc);

        for(int i = 0; i < 3; ++i)
        {
            Matrix2 gseHessian = _gradientSubExprHessian(i, bc);
            h[i + 3] = 4 * _edgeSubExprHessian(i, bc)
                     + 4 * gseHessian
                     - 12 * bubbleHessian;
            h[i + 6] = _gradientFactor(i) * gseHessian;
        }

        for(int i = 0; i < 3; ++i)
        {
            unsigned i1 = (i+1) % 3;
            unsigned i2 = (i+2) % 3;
            Matrix2 v1 = h[i1 + 6];
            Matrix2 v2 = h[i2 + 6];
            h[i] = _vertexSubExprHessian(i, bc)
                 + 3 * bubbleHessian
                 + _vertexGradientFactor(i, i1) * v1
                 + _vertexGradientFactor(i, i2) * v2;
        }
    }

    inline void hessian(const Vector& p, Matrix2* h) const
    {
        hessian(barycentricCoordinates(p), h);
    }

    inline Matrix2 hessian(unsigned bi, const Vector& p) const
    {
        Vector3 bc = barycentricCoordinates(p);
        return hessian(bi, bc);
    }

    inline Matrix2 hessian(unsigned bi, const Vector3& bc) const
    {
        assert(bi < 9);

        Matrix2 h;
        unsigned i = bi % 3;
        switch(bi/3)
        {
        case 0:
        {
            unsigned i1 = (i+1) % 3;
            unsigned i2 = (i+2) % 3;
            h = _vertexSubExprHessian(i, bc)
              + 3 * _bubbleHessian(bc)
              + _vertexGradientFactor(i, i1) * _gradientFactor(i1) * _gradientSubExprHessian(i1, bc)
              + _vertexGradientFactor(i, i2) * _gradientFactor(i2) * _gradientSubExprHessian(i2, bc);
            break;
        }
        case 1:
        {
            h =  4 * _edgeSubExprHessian(i, bc)
              +  4 * _gradientSubExprHessian(i, bc)
              - 12 * _bubbleHessian(bc);
            break;
        }
        case 2:
        {
            h = _gradientFactor(i) * _gradientSubExprHessian(i, bc);
            break;
        }
        }

        return h;
    }


public:
    inline Scalar _gradientFactor(unsigned bi) const
    {
        return - m_2delta / edgeLength(bi);
    }

    inline Scalar _vertexGradientFactor(unsigned bi, unsigned ei) const
    {
        return dldn(bi, ei) + dldn(ei, ei) / 2;
    }

    inline Scalar _bubble(const Vector3& bc) const
    {
        return bc.prod();
    }

    inline Vector _bubbleGradient(const Vector3& bc) const
    {
        return Base::jacobian().row(0) * bc(1) * bc(2)
             + Base::jacobian().row(1) * bc(2) * bc(0)
             + Base::jacobian().row(2) * bc(0) * bc(1);
    }

    inline Matrix2 _bubbleHessian(const Vector3& bc) const
    {
        Matrix2 h;
        for(int d0 = 0; d0 < 2; ++d0)
        {
            for(int d1 = d0; d1 < 2; ++d1)
            {
                h(d0, d1) = Base::jacobian()(0, d0) * Base::jacobian()(1, d1) * bc(2)
                          + Base::jacobian()(0, d0) * Base::jacobian()(2, d1) * bc(1)
                          + Base::jacobian()(1, d0) * Base::jacobian()(0, d1) * bc(2)
                          + Base::jacobian()(1, d0) * Base::jacobian()(2, d1) * bc(0)
                          + Base::jacobian()(2, d0) * Base::jacobian()(0, d1) * bc(1)
                          + Base::jacobian()(2, d0) * Base::jacobian()(1, d1) * bc(0);
            }
        }
        h(1, 0) = h(0, 1);
        return h;
    }

    inline Scalar _vertexSubExpr(unsigned i, const Vector3& bc) const
    {
        return bc(i) * (bc(i) - .5) * (bc(i) + 1);
    }

    inline Vector _vertexSubExprGradient(unsigned i, const Vector3& bc) const
    {
        return Base::jacobian().row(i) * (3 * bc[i] * bc[i] + bc[i] - .5);
    }

    inline Matrix2 _vertexSubExprHessian(unsigned i, const Vector3& bc) const
    {
        Matrix2 h;
        for(int d0 = 0; d0 < 2; ++d0)
        {
            for(int d1 = d0; d1 < 2; ++d1)
            {
                h(d0, d1) = Base::jacobian()(i, d0)
                          * Base::jacobian()(i, d1)
                          * (6 * bc(i) + 1);
            }
        }
        h(1, 0) = h(0, 1);
        return h;
    }

    inline Scalar _edgeSubExpr(unsigned i, const Vector3& bc) const
    {
        return bc((i+1)%3) * bc((i+2)%3);
    }

    inline Vector _edgeSubExprGradient(unsigned i, const Vector3& bc) const
    {
        unsigned i1 = (i+1) % 3;
        unsigned i2 = (i+2) % 3;
        return Base::jacobian().row(i1) * bc(i2)
             + Base::jacobian().row(i2) * bc(i1);
    }

    inline Matrix2 _edgeSubExprHessian(unsigned i, const Vector3& /*bc*/) const
    {
        unsigned i1 = (i+1) % 3;
        unsigned i2 = (i+2) % 3;
        Matrix2 h;
        for(int d0 = 0; d0 < 2; ++d0)
        {
            for(int d1 = d0; d1 < 2; ++d1)
            {
                h(d0, d1) = Base::jacobian()(i1, d0) * Base::jacobian()(i2, d1)
                          + Base::jacobian()(i2, d0) * Base::jacobian()(i1, d1);
            }
        }
        h(1, 0) = h(0, 1);
        return h;
    }

    inline Scalar _gradientSubExpr(unsigned i, const Vector3& bc) const
    {
        return bc(i) * (2.*bc(i) - 1.) * (bc(i) - 1.);
    }

    inline Vector _gradientSubExprGradient(unsigned i, const Vector3& bc) const
    {
        return Base::jacobian().row(i) * (6 * bc(i) * bc(i) - 6 * bc(i) + 1);
    }

    inline Matrix2 _gradientSubExprHessian(unsigned i, const Vector3& bc) const
    {
        Matrix2 h;
        for(int d0 = 0; d0 < 2; ++d0)
        {
            for(int d1 = d0; d1 < 2; ++d1)
            {
                h(d0, d1) = Base::jacobian()(i, d0)
                          * Base::jacobian()(i, d1)
                          * (12 * bc(i) - 6);
            }
        }
        h(1, 0) = h(0, 1);
        return h;
    }

protected:
    void computeFromPoints()
    {
        Matrix2x3 vs;
        for(int i = 0; i < 3; ++i)
            vs.col(i) = point(i, 2) - point(i, 1);

        for(int i = 0; i < 3; ++i)
            m_eLen(i) = vs.col(i).norm();

//        m_rot.row(0) = vs.col(2) / edgeLength(2);
//        m_rot(1, 0) = -m_rot(0, 1);
//        m_rot(1, 1) =  m_rot(0, 0);

//        Matrix2x3 pts = m_rot * m_points;

        for(int ni = 0; ni < 3; ++ni)
            for(int li = 0; li < 3; ++li)
                m_dldn(li, ni) =
                        vs.col(li).dot(vs.col(ni)) / (m_2delta * edgeLength(ni));
    }


protected:
    using Base::m_points;
    using Base::m_2delta;
    using Base::m_lbf;

//    Matrix2 m_rot;

    Vector3 m_eLen;
    Matrix3 m_dldn;
};


} // namespace Vitelotte


#endif
