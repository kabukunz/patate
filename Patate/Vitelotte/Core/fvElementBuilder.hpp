/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


#include <Eigen/Dense>

#include "fvElementBuilder.h"


namespace Vitelotte
{


template < typename _Scalar >
class _LinearElement
{
public:
    typedef _Scalar Scalar;

    typedef Eigen::Matrix<Scalar, 2, 1> Vector;
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;

    typedef Eigen::Matrix<Scalar, 2, 2> Matrix2;
    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

    typedef Eigen::Matrix<Scalar, 3, 2> Matrix3x2;
    typedef Eigen::Matrix<Scalar, 2, 3> Matrix2x3;

public:
    inline _LinearElement(const Vector& p0, const Vector& p1, const Vector& p2)
    {
        m_points.col(0) = p0;
        m_points.col(1) = p1;
        m_points.col(2) = p2;

        computeFromPoints();
    }

    inline Vector point(unsigned pi, unsigned offset=0) const
    {
        assert(pi < 3 && offset < 3);
        return m_points.col((pi + offset) % 3);
    }

    inline Scalar doubleArea() const { return m_2delta; }

    inline Vector3 eval(const Vector& p) const
    {
        return m_lbf * (Vector3() << p, 1).finished();
    }

    inline const Eigen::Block<const Matrix3, 3, 2> jacobian(const Vector& /*p*/ = Vector()) const
    {
        return m_lbf.template block<3, 2>(0, 0);
    }

    void computeFromPoints()
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

#ifndef DNDEBUG
//        std::cout << "Compute Linear:\n";
//        for(int i = 0; i < 3; ++i)
//            std::cout << "  p" << i << ": " << point(i).transpose()
//                      << " (" << eval(point(i)).transpose() << ")\n";
//        std::cout << "  2delta: " << m_2delta << "\n";

//        std::cout << "J:\n" << jacobian() << "\n";
#endif
    }

protected:
    Matrix2x3 m_points;

    Scalar m_2delta;
    Matrix3 m_lbf;
};


template < typename _Scalar >
class _FVElement : public _LinearElement<_Scalar>
{
public:
    typedef _Scalar Scalar;
    typedef _LinearElement<Scalar> Base;

    typedef Eigen::Matrix<Scalar, 2, 1> Vector;
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<Scalar, 9, 1> Vector9;

    typedef Eigen::Matrix<Scalar, 2, 2> Matrix2;
    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

    typedef Eigen::Matrix<Scalar, 2, 3> Matrix2x3;
    typedef Eigen::Matrix<Scalar, 9, 2> Vector9x2;


public:
    inline _FVElement(const Vector& p0, const Vector& p1, const Vector& p2)
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

/*    inline Scalar gradientBasis(unsigned i, const Vector& p) const
    {
        Vector3 bc = barycentricCoords(p);
        return _gradientFactor(i) * _gradientSubExpr(i, bc);
    }

    inline Scalar edgeBasis(unsigned i, const Vector& p) const
    {
        Vector3 bc = barycentricCoords(p);
        return    4 * _edgeSubExpr(i, bc)
                + 4 * _gradientSubExpr(i, bc)
                - 12 * _bubble(bc);
    }

    inline Scalar vertexBasis(unsigned i, const Vector& p) const
    {
        Vector3 bc = barycentricCoords(p);
        unsigned i1 = (i+1) % 3;
        unsigned i2 = (i+2) % 3;
        return    _vertexSubExpr(i, bc)
                + 3 * _bubble(bc)
                + _vertexGradientFactor(i, i1) * _gradientFactor(i1) * _gradientSubExpr(i1, bc)
                + _vertexGradientFactor(i, i2) * _gradientFactor(i2) * _gradientSubExpr(i2, bc);
    }*/

    inline Vector9 eval(const Vector& p) const
    {
        Vector9 eb;
        Vector3 bc = Base::eval(p);
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

    inline Vector9x2 gradient(const Vector& p) const
    {
        Vector9x2 grad;
        Vector3 bc = Base::eval(p);
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
        hessian(Base::eval(p), h);
    }

    void computeFromPoints()
    {
        Matrix2x3 vs;
        for(int i = 0; i < 3; ++i)
            vs.col(i) = point(i, 2) - point(i, 1);

        for(int i = 0; i < 3; ++i)
            m_eLen(i) = vs.col(i).norm();

        m_rot.row(0) = vs.col(2) / edgeLength(2);
        m_rot(1, 0) = -m_rot(0, 1);
        m_rot(1, 1) =  m_rot(0, 0);

        Matrix2x3 pts = m_rot * m_points;

        for(int ni = 0; ni < 3; ++ni)
            for(int li = 0; li < 3; ++li)
                m_dldn(li, ni) =
                        vs.col(li).dot(vs.col(ni)) / (m_2delta * edgeLength(ni));
    }


protected:
    using Base::m_points;
    using Base::m_2delta;
    using Base::m_lbf;

    Matrix2 m_rot;

    Vector3 m_eLen;
    Matrix3 m_dldn;
};


template < typename _Scalar >
class _FVElementFlat : public _FVElement<_Scalar>
{
public:
    typedef _Scalar Scalar;
    typedef _FVElement<Scalar> Base;

    typedef Eigen::Matrix<Scalar, 2, 1> Vector;
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<Scalar, 9, 1> Vector9;

    typedef Eigen::Matrix<Scalar, 2, 2> Matrix2;
    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

    typedef Eigen::Matrix<Scalar, 2, 3> Matrix2x3;
    typedef Eigen::Matrix<Scalar, 9, 2> Matrix9x2;


public:
    inline _FVElementFlat(const Vector& p0, const Vector& p1, const Vector& p2)
        : Base(p0, p1, p2)
    {
    }

    using Base::point;
    using Base::doubleArea;
    using Base::edgeLength;
    using Base::dldn;

    using Base::_gradientFactor;
    using Base::_vertexGradientFactor;
    using Base::_bubble;
    using Base::_bubbleGradient;
    using Base::_bubbleHessian;
    using Base::_vertexSubExpr;
    using Base::_vertexSubExprGradient;
    using Base::_vertexSubExprHessian;
    using Base::_edgeSubExpr;
    using Base::_edgeSubExprGradient;
    using Base::_edgeSubExprHessian;
    using Base::_gradientSubExpr;
    using Base::_gradientSubExprGradient;
    using Base::_gradientSubExprHessian;

//    using Base::eval;
//    using Base::gradient;
//    using Base::hessian;

    Matrix9x2 _diffP0AlongEdges() const
    {
        return (Matrix9x2() <<
                -1.0L/2.0L*(2*m_dldn(0, 1)*m_2delta + m_dldn(1, 1)*m_2delta + 7*m_eLen(1))/(m_eLen(1)*m_eLen(2)),
                -1.0L/2.0L*(2*m_dldn(0, 2)*m_2delta + m_dldn(2, 2)*m_2delta + 7*m_eLen(2))/(m_eLen(1)*m_eLen(2)),
                (1.0L/2.0L)*(m_dldn(0, 0)*m_2delta + 2*m_dldn(1, 0)*m_2delta - m_eLen(0))/(m_eLen(0)*m_eLen(2)),
                (1.0L/2.0L)*m_2delta*(m_dldn(0, 0)*m_eLen(2) + 2*m_dldn(1, 0)*m_eLen(2) - 2*m_dldn(1, 2)*m_eLen(0) - m_dldn(2, 2)*m_eLen(0))/(m_eLen(0)*m_eLen(1)*m_eLen(2)),
                (1.0L/2.0L)*m_2delta*(m_dldn(0, 0)*m_eLen(1) - m_dldn(1, 1)*m_eLen(0) + 2*m_dldn(2, 0)*m_eLen(1) - 2*m_dldn(2, 1)*m_eLen(0))/(m_eLen(0)*m_eLen(1)*m_eLen(2)),
                (1.0L/2.0L)*(m_dldn(0, 0)*m_2delta + 2*m_dldn(2, 0)*m_2delta - m_eLen(0))/(m_eLen(0)*m_eLen(1)),
                -4/m_eLen(2),
                -4/m_eLen(1),
                4/m_eLen(2),
                4/m_eLen(1),
                4/m_eLen(2),
                4/m_eLen(1),
                m_2delta/(m_eLen(0)*m_eLen(2)),
                m_2delta/(m_eLen(0)*m_eLen(1)),
                -m_2delta/(m_eLen(1)*m_eLen(2)),
                0,
                0,
                -m_2delta/(m_eLen(1)*m_eLen(2))
        ).finished();
    }

    inline Vector9 eval(const Vector& p) const
    {
        Vector9 fv = Base::eval(p);
        Matrix9x2 diffP0 = _diffP0AlongEdges();

        fv[7] /= diffP0(7, 0);
        fv[8] /= diffP0(8, 1);
        for(unsigned i = 0; i < 7; ++i)
        {
            fv[i] -= diffP0(i, 0) * fv[7];
            fv[i] -= diffP0(i, 1) * fv[8];
        }

        std::swap(fv[7], fv[8]);

        return fv;
    }

    inline void hessian(const Vector3& bc, Matrix2* h) const
    {
        Base::hessian(bc, h);
        Matrix9x2 diffP0 = _diffP0AlongEdges();

        h[7] /= diffP0(7, 0);
        h[8] /= diffP0(8, 1);
        for(unsigned i = 0; i < 7; ++i)
        {
            h[i] -= diffP0(i, 0) * h[7];
            h[i] -= diffP0(i, 1) * h[8];
        }

        std::swap(h[7], h[8]);
    }

    inline void hessian(const Vector& p, Matrix2* h) const
    {
        hessian(Base::Base::eval(p), h);
    }

private:
    using Base::m_points;
    using Base::m_2delta;
    using Base::m_lbf;

    using Base::m_rot;

    using Base::m_eLen;
    using Base::m_dldn;
};


template < class _Mesh, typename _Scalar >
FVElementBuilder<_Mesh, _Scalar>::FVElementBuilder(Scalar sigma)
  : m_sigma(sigma)
{
}

template < class _Mesh, typename _Scalar >
unsigned
FVElementBuilder<_Mesh, _Scalar>::
    nCoefficients(const Mesh& /*mesh*/, Face /*element*/) const
{
    return 81;
}


template < class _Mesh, typename _Scalar >
template < typename InIt >
void
FVElementBuilder<_Mesh, _Scalar>::
    addCoefficients(InIt& it, const Mesh& mesh, Face element)
{
    if(mesh.valence(element) != 3)
    {
        error(STATUS_ERROR, "Non-triangular face");
        return;
    }

    // TODO: remove some code duplication by moving stuff here

//    typename Mesh::HalfedgeAroundFaceCirculator hit = mesh.halfedges(element);
//    bool flat = false;
//    for(int i = 0; i < 3; ++i)
//    {
//        if(mesh.hasFlatGradient(mesh.toVertex(*hit)))
//        {
//            flat = true;
//            break;
//        }
//        ++hit;
//    }

//    if(flat)
//        processFV1ElementFlat(it, mesh, element);
//    else
        processFV1Element(it, mesh, element);
}

template < class _Mesh, typename _Scalar >
template < typename InIt >
void
FVElementBuilder<_Mesh, _Scalar>::
    processFV1Element(InIt& it, const Mesh& mesh, Face element)
{
    int method = 2;

    typedef Eigen::Matrix<Scalar, 9, 9> Matrix9;
    Matrix9 smOld;
    Matrix9 smNew;

    int nodes[9];
    if(method & 1)
    {
        typename Mesh::HalfedgeAroundFaceCirculator hit = mesh.halfedges(element);

        bool orient[3];
        Vector v[3];
        --hit;
        for(int i = 0; i < 3; ++i)
        {
            v[i] = (mesh.position(mesh.toVertex(*hit)) -
                    mesh.position(mesh.fromVertex(*hit))).template cast<Scalar>();
            orient[i] = mesh.halfedgeOrientation(*hit);
            nodes[3+i] = mesh.edgeValueNode(*hit).idx();
            nodes[6+i] = mesh.edgeGradientNode(*hit).idx();
            ++hit;
            nodes[i] = mesh.toVertexValueNode(*hit).idx();
        }

        for(int i = 0; i < 9; ++i)
        {
            if(nodes[i] < 0)
            {
                error(STATUS_ERROR, "Invalid node");
                return;
            }
        }

        Vector p[] = {
            Vector::Zero(),
            v[2],
            -v[1]
        };

        Scalar area = det2(v[0], v[1]) / 2.;

        if(area <= 0)
        {
            error(STATUS_WARNING, "Degenerated or reversed triangle");
        }

        Vector3 a, b, c, d, l;
        for(size_t i0 = 0; i0 < 3; ++i0)
        {
            size_t i1 = (i0+1)%3;
            size_t i2 = (i0+2)%3;
            a(i0) = det2(p[i1], p[i2]);
            b(i0) = -v[i0](1);
            c(i0) = v[i0](0);
            d(i0) = .5 - v[i0].dot(-v[i2]) / v[i0].squaredNorm();
            l(i0) = v[i0].norm();
        }

    // TODO : check that debug info
    /*#ifndef _DEBUG
            for(size_t i = 0; i < 3; ++i)
        {
                    for(size_t j = 0; j < 3; ++j)
            {
                            Scalar v = a(i) + b(i) * p[j](0) + c(i) * p[j](1);
                            Scalar r = (i==j)? 2.*area: 0.;
                            assert(std::abs(v - r) < 1.e-8);
                    }

                    assert(std::abs(l(i) - std::sqrt(b(i)*b(i) + c(i)*c(i))) < 1.e-8);
            }
    #endif*/

        Vector3 dx2[9];
        Vector3 dy2[9];
        Vector3 dxy[9];

        for(size_t i = 0; i < 3; ++i)
        {
        // 25%:
            dx2[i+0] = funcVertexBasis(i, Deriv_XX, a, b, c, d, l, area);
            dx2[i+3] = funcMidpointBasis(i, Deriv_XX, a, b, c, d, l, area);
            dx2[i+6] = funcMidpointDerivBasis(i, Deriv_XX, a, b, c, d, l, area);
            dy2[i+0] = funcVertexBasis(i, Deriv_YY, a, b, c, d, l, area);
            dy2[i+3] = funcMidpointBasis(i, Deriv_YY, a, b, c, d, l, area);
            dy2[i+6] = funcMidpointDerivBasis(i, Deriv_YY, a, b, c, d, l, area);
            dxy[i+0] = funcVertexBasis(i, Deriv_XY, a, b, c, d, l, area);
            dxy[i+3] = funcMidpointBasis(i, Deriv_XY, a, b, c, d, l, area);
            dxy[i+6] = funcMidpointDerivBasis(i, Deriv_XY, a, b, c, d, l, area);
        };

        for(size_t i = 0; i < 9; ++i)
        {
            for(size_t j = i; j < 9; ++j)
            {
                EIGEN_ASM_COMMENT("MYBEGIN");

                Vector6 basis = multBasis(dx2[i]+dy2[i], dx2[j]+dy2[j])
                                        + (1.-m_sigma) * (2. * multBasis(dxy[i], dxy[j])
                                                        - multBasis(dx2[i], dy2[j])
                        - multBasis(dy2[i], dx2[j]));

                Scalar value = integrateQuadTriangle(v, basis, area);

                EIGEN_ASM_COMMENT("MYEND");

                if((i < 6 || orient[i%3]) != (j < 6 || orient[j%3]))
                {
                    value *= -1;
                }

//                *(it++) = Triplet(nodes[i], nodes[j], value);
//                if(i != j)
//                    *(it++) = Triplet(nodes[j], nodes[i], value);
                smOld(i, j) = value;
                smOld(j, i) = value;
            }
        }
    }

    if(method & 2)
    {
        typename Mesh::HalfedgeAroundFaceCirculator hit = mesh.halfedges(element);

        typename Mesh::HalfedgeAroundFaceCirculator hend = hit;
        do ++hit;
        while(mesh.toVertex(*hit).idx() != 0 && hit != hend);
        bool flat = mesh.toVertex(*hit).idx() == 0;
//        flat = false;

        bool orient[3];
        Vector p[3];
        --hit;
        for(int i = 0; i < 3; ++i)
        {
            orient[i] = mesh.halfedgeOrientation(*hit);
            nodes[3+i] = mesh.edgeValueNode(*hit).idx();
            nodes[6+i] = mesh.edgeGradientNode(*hit).idx();
            ++hit;
            nodes[i] = mesh.toVertexValueNode(*hit).idx();
            p[i] = mesh.position(mesh.toVertex(*hit)).template cast<Scalar>();
        }

        for(int i = 0; i < 9; ++i)
        {
            if(nodes[i] < 0)
            {
                error(STATUS_ERROR, "Invalid node");
                return;
            }
        }

        typedef _FVElement<Scalar> Elem;
        Elem elem(p[0], p[1], p[2]);

        if(elem.doubleArea() <= 0)
        {
            error(STATUS_WARNING, "Degenerated or reversed triangle");
        }

        typedef Eigen::Array<Scalar, 3, 1> Array3;
        Array3 dx2[9];
        Array3 dy2[9];
        Array3 dxy[9];
        for(int pi = 0; pi < 3; ++pi)
        {
            Vector3 bc((pi == 0)? 0: .5,
                       (pi == 1)? 0: .5,
                       (pi == 2)? 0: .5);
            typename Elem::Matrix2 hessians[9];
            if(!flat)
            {
                elem.hessian(bc, hessians);
            }
            else
            {
                typedef _FVElementFlat<Scalar> ElemFlat;
                ElemFlat elemFlat(p[0], p[1], p[2]);
                elemFlat.hessian(bc, hessians);
                orient[7] = 1;
                orient[8] = 1;

                Scalar delta = 1.e-6;

                Vector dx(delta/2, 0);
                Vector dy(0, delta/2);
                for(unsigned bi = 0; bi < 9; ++bi)
                {
                    hessians[bi](0, 0) = elemFlat.eval(elemFlat.point(pi) + dx + dx)(bi)
                                       - elemFlat.eval(elemFlat.point(pi) + dx - dx)(bi)
                                       - elemFlat.eval(elemFlat.point(pi) - dx + dx)(bi)
                                       + elemFlat.eval(elemFlat.point(pi) - dx - dx)(bi);
                    hessians[bi](0, 1) = elemFlat.eval(elemFlat.point(pi) + dx + dy)(bi)
                                       - elemFlat.eval(elemFlat.point(pi) + dx - dy)(bi)
                                       - elemFlat.eval(elemFlat.point(pi) - dx + dy)(bi)
                                       + elemFlat.eval(elemFlat.point(pi) - dx - dy)(bi);
                    hessians[bi](1, 0) = hessians[bi](0, 1);
                    hessians[bi](1, 1) = elemFlat.eval(elemFlat.point(pi) + dy + dy)(bi)
                                       - elemFlat.eval(elemFlat.point(pi) + dy - dy)(bi)
                                       - elemFlat.eval(elemFlat.point(pi) - dy + dy)(bi)
                                       + elemFlat.eval(elemFlat.point(pi) - dy - dy)(bi);
                    hessians[bi] /= delta * delta;
                }

                typedef Eigen::Matrix<Scalar, 9, 2> Matrix9x2;
                Vector dv1 = (elemFlat.point(1) - elemFlat.point(0)).normalized() * (delta/2);
                Vector dv2 = (elemFlat.point(2) - elemFlat.point(0)).normalized() * (delta/2);
                Matrix9x2 testDiffP0;
                testDiffP0 <<
                    (elemFlat.eval(elemFlat.point(0) + dv2) - elemFlat.eval(elemFlat.point(0) - dv2)) / delta,
                    (elemFlat.eval(elemFlat.point(0) + dv1) - elemFlat.eval(elemFlat.point(0) - dv1)) / delta;
                std::cout << "Test diff p0:\n" << testDiffP0.transpose() << "\n";
            }

            for(int bi = 0; bi < 9; ++bi)
            {
                dx2[bi](pi) = hessians[bi](0, 0);
                dy2[bi](pi) = hessians[bi](1, 1);
                dxy[bi](pi) = hessians[bi](0, 1);
            }
        }

        for(size_t i = 0; i < 9; ++i)
        {
            for(size_t j = i; j < 9; ++j)
            {
                EIGEN_ASM_COMMENT("MYBEGIN");

                Array3 quadPointValue =
                        (dx2[i]+dy2[i]) * (dx2[j]+dy2[j])
                      + (1.-m_sigma) * (
                            2. * dxy[i] * dxy[j]
                          - dx2[i] * dy2[j]
                          - dx2[j] * dy2[i]);

                Scalar value = quadPointValue.sum() * (elem.doubleArea() / 6);

                EIGEN_ASM_COMMENT("MYEND");

                if((i < 6 || orient[i%3]) != (j < 6 || orient[j%3]))
                {
                    value *= -1;
                }

//                *(it++) = Triplet(nodes[i], nodes[j], value);
//                if(i != j)
//                    *(it++) = Triplet(nodes[j], nodes[i], value);
                smNew(i, j) = value;
                smNew(j, i) = value;
            }
        }

        if(flat)
        {
//            typedef Eigen::Matrix<Scalar, 9, 1> Vector9;
//            Vector9 fde1;
//            Vector9 fde2;
//            fde1 <<
//                -1.0L/2.0L*(2*elem.doubleArea()*elem.dldn(0, 1) + elem.doubleArea()*elem.dldn(1, 1) + 7*elem.edgeLength(1))/(elem.edgeLength(1)*elem.edgeLength(2)),
//                (1.0L/2.0L)*(elem.doubleArea()*elem.dldn(0, 0) + 2*elem.doubleArea()*elem.dldn(1, 0) - elem.edgeLength(0))/(elem.edgeLength(0)*elem.edgeLength(2)),
//                (1.0L/2.0L)*elem.doubleArea()*(elem.dldn(0, 0)*elem.edgeLength(1) - elem.dldn(1, 1)*elem.edgeLength(0) + 2*elem.dldn(2, 0)*elem.edgeLength(1) - 2*elem.dldn(2, 1)*elem.edgeLength(0))/(elem.edgeLength(0)*elem.edgeLength(1)*elem.edgeLength(2)),
//                -4/elem.edgeLength(2),
//                4/elem.edgeLength(2),
//                4/elem.edgeLength(2),
//                elem.doubleArea()/(elem.edgeLength(0)*elem.edgeLength(2)),
//                -elem.doubleArea()/(elem.edgeLength(1)*elem.edgeLength(2)),
//                0;
//            fde2 <<
//                -1.0L/2.0L*(2*elem.doubleArea()*elem.dldn(0, 2) + elem.doubleArea()*elem.dldn(2, 2) + 7*elem.edgeLength(2))/(elem.edgeLength(1)*elem.edgeLength(2)),
//                (1.0L/2.0L)*elem.doubleArea()*(elem.dldn(0, 0)*elem.edgeLength(2) + 2*elem.dldn(1, 0)*elem.edgeLength(2) - 2*elem.dldn(1, 2)*elem.edgeLength(0) - elem.dldn(2, 2)*elem.edgeLength(0))/(elem.edgeLength(0)*elem.edgeLength(1)*elem.edgeLength(2)),
//                (1.0L/2.0L)*(elem.doubleArea()*elem.dldn(0, 0) + 2*elem.doubleArea()*elem.dldn(2, 0) - elem.edgeLength(0))/(elem.edgeLength(0)*elem.edgeLength(1)),
//                -4/elem.edgeLength(1),
//                4/elem.edgeLength(1),
//                4/elem.edgeLength(1),
//                elem.doubleArea()/(elem.edgeLength(0)*elem.edgeLength(1)),
//                0,
//                -elem.doubleArea()/(elem.edgeLength(1)*elem.edgeLength(2));

//            smNew.row(7) += fde1;
////            smNew.col(7) = fde1;
//            smNew.row(8) += fde2;
////            smNew.col(8) = fde2;

            std::cout << "Flat elem:\n";
            std::cout << "  p0: " << elem.point(0).transpose() << "\n";
            std::cout << "  p1: " << elem.point(1).transpose() << "\n";
            std::cout << "  p2: " << elem.point(2).transpose() << "\n";
            std::cout << "  n8: " << mesh.nodeValue(typename Mesh::Node(nodes[8])).transpose() << "\n";
            std::cout << "  n7: " << mesh.nodeValue(typename Mesh::Node(nodes[7])).transpose() << "\n";
            std::cout << "  Stiffness matrix:\n" << smNew << "\n";
        }
    }

    Matrix9& sm = (method & 2)? smNew: smOld;

    for(size_t i = 0; i < 9; ++i)
    {
        for(size_t j = 0; j < 9; ++j)
        {
            *(it++) = Triplet(nodes[i], nodes[j], sm(i, j));
        }
    }

    if(method == 3 && element.idx() < 6)
    {
        std::cout << "Stiffness diff:\n" << (smNew.array() / 2) - smOld.array() << "\n";
        std::cout << "Stiffness ratio:\n" << smNew.array() / smOld.array() << "\n";
    }
}


template < class _Mesh, typename _Scalar >
typename FVElementBuilder<_Mesh, _Scalar>::Vector3
FVElementBuilder<_Mesh, _Scalar>::funcVertexBasis(
        const int _i1, int _deriv, const Vector3& _a, const Vector3& _b,
        const Vector3& _c, const Vector3& _d, const Vector3& /*_l*/,
        const Scalar _area) const
{
    const size_t i2 = (_i1+1)%3;
    const size_t i3 = (_i1+2)%3;

    const Vector3 d1 = (_deriv&Deriv_0_Y)? _c: _b;
    const Vector3 d2 = (_deriv&Deriv_1_Y)? _c: _b;

    Scalar factorA = d1(_i1)*d2(_i1)/(4.*_area*_area*_area);
    Scalar factorB = 3./(8.*_area*_area*_area);

    Scalar factorC = -3.*_d(i2)*d1(i2)*d2(i2)/(2.*_area*_area*_area);
    Scalar factorD = 3.*_d(i3)*d1(i3)*d2(i3)/(2.*_area*_area*_area);

    Vector3 comb(d1(i2)*d2(i3) + d1(i3)*d2(i2),
                    d1(_i1)*d2(i3) + d1(i3)*d2(_i1),
                    d1(_i1)*d2(i2) + d1(i2)*d2(_i1));

    return factorA * Vector3(3.*_a(_i1) + _area, 3.*_b(_i1), 3.*_c(_i1))
            +  factorB * Vector3(
            (_a(_i1)*comb(0) + _a(i2)*comb(1) + _a(i3)*comb(2)),
            (_b(_i1)*comb(0) + _b(i2)*comb(1) + _b(i3)*comb(2)),
            (_c(_i1)*comb(0) + _c(i2)*comb(1) + _c(i3)*comb(2)))
            +  factorC * Vector3(_a(i2) - _area, _b(i2), _c(i2))
            +  factorD * Vector3(_a(i3) - _area, _b(i3), _c(i3));
}

template < class _Mesh, typename _Scalar >
typename FVElementBuilder<_Mesh, _Scalar>::Vector3
FVElementBuilder<_Mesh, _Scalar>::funcMidpointBasis(
        const int _i1, int _deriv, const Vector3& _a, const Vector3& _b,
        const Vector3& _c, const Vector3& /*_d*/, const Vector3& /*_l*/,
        const Scalar _area) const
{
    const size_t i2 = (_i1+1)%3;
    const size_t i3 = (_i1+2)%3;

    const Vector3 d1 = (_deriv&Deriv_0_Y)? _c: _b;
    const Vector3 d2 = (_deriv&Deriv_1_Y)? _c: _b;

    Scalar factorA = 6.*d1(_i1)*d2(_i1)/(_area*_area*_area);
    Scalar factorB = -3./(2.*_area*_area*_area);

    Vector3 comb(d1(i2)*d2(i3) + d1(i3)*d2(i2),
                    d1(_i1)*d2(i3) + d1(i3)*d2(_i1),
                    d1(_i1)*d2(i2) + d1(i2)*d2(_i1));

    return Vector3(comb(0) / (_area*_area), 0., 0.)
            +  factorA * Vector3(_a(_i1) - _area, _b(_i1), _c(_i1))
            +  factorB * Vector3(
            (_a(_i1)*comb(0) + _a(i2)*comb(1) + _a(i3)*comb(2)),
            (_b(_i1)*comb(0) + _b(i2)*comb(1) + _b(i3)*comb(2)),
            (_c(_i1)*comb(0) + _c(i2)*comb(1) + _c(i3)*comb(2)));
}

template < class _Mesh, typename _Scalar >
typename FVElementBuilder<_Mesh, _Scalar>::Vector3
FVElementBuilder<_Mesh, _Scalar>::funcMidpointDerivBasis(
        const int _i1, int _deriv, const Vector3& _a, const Vector3& _b,
        const Vector3& _c, const Vector3& /*_d*/, const Vector3& _l,
        const Scalar _area) const
{
    const Vector3& d1 = (_deriv & Deriv_0_Y) ? _c : _b;
    const Vector3& d2 = (_deriv & Deriv_1_Y) ? _c : _b;

    Scalar factorA = -3. * d1(_i1) * d2(_i1) / (_l(_i1) * _area * _area);

    return factorA * Vector3(_a(_i1) - _area, _b(_i1), _c(_i1));
}

template < class _Mesh, typename _Scalar >
typename FVElementBuilder<_Mesh, _Scalar>::Scalar
FVElementBuilder<_Mesh, _Scalar>::integrateQuadTriangle(
        const Vector* _v, const Vector6& _f, Scalar _area) const
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

template < class _Mesh, typename _Scalar >
typename FVElementBuilder<_Mesh, _Scalar>::Vector6
FVElementBuilder<_Mesh, _Scalar>::
    multBasis(const Vector3& _a, const Vector3& _b) const
{
    return  (
        Vector6() << _a(0)*_b(0), _a(0)*_b(1) + _a(1)*_b(0), _a(0)*_b(2) + _a(2)*_b(0),
                     _a(1)*_b(1), _a(1)*_b(2) + _a(2)*_b(1), _a(2)*_b(2)
    ).finished();
}


}
