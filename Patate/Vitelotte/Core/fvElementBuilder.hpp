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

    inline Matrix2 _edgeSubExprHessian(unsigned i, const Vector3& bc) const
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
        return bc(i) * (2*bc(i) - 1) * (bc(i) - 1);
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

    inline Scalar gradientBasis(unsigned i, const Vector& p) const
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
    }

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
        hessian(Base::eval(p));
    }

    Vector9 _integrateNormalDerivativeOverSegment(
            const Vector& p1, const Vector& p2, unsigned nsamples, Scalar delta) const
    {
        Vector9 eb = Vector9::Zero();
        Vector v = p2 - p1;
        Vector n(-v.y(), v.x());
        n.normalize();
        for(int i = 0; i < nsamples; ++i)
        {
            Vector lp = p1 + (Scalar(i + .5) / Scalar(nsamples)) * v;

//            std::cout << "indos: " << (lp + n*delta/2).transpose() << " - "
//                      << (lp - n*delta/2).transpose() << "\n";
//            std::cout << "indos +: " << evalBasis(lp + n*delta/2).transpose() << "\n";
//            std::cout << "indos -: " << evalBasis(lp - n*delta/2).transpose() << "\n";
            eb += eval(lp + n*delta/2);
            eb -= eval(lp - n*delta/2);
        }
        return eb / (delta * nsamples);
    }

    Vector9 _integrateNormalDerivativeOverSegment2(
            const Vector& p1, const Vector& p2, unsigned nsamples) const
    {
        Vector9 eb = Vector9::Zero();
        Vector v = p2 - p1;
        Vector n(-v.y(), v.x());
        n.normalize();
        for(int i = 0; i < nsamples; ++i)
        {
            Vector lp = p1 + (Scalar(i + .5) / Scalar(nsamples)) * v;

//            std::cout << "indos: " << (lp + n*delta/2).transpose() << " - "
//                      << (lp - n*delta/2).transpose() << "\n";
//            std::cout << "indos +: " << evalBasis(lp + n*delta/2).transpose() << "\n";
//            std::cout << "indos -: " << evalBasis(lp - n*delta/2).transpose() << "\n";
            eb += gradient(lp) * n;
        }
        return eb / nsamples;
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


#ifndef DNDEBUG
//        Matrix2x3 vs_;
//        for(int i = 0; i < 3; ++i)
//            vs_.col(i) = pts.col((i+2)%3) - pts.col((i+1)%3);

//        std::cout << "Compute FV:\n";
//        for(int i = 0; i < 3; ++i)
//            std::cout << "  v" << i << ": " << vs.col(i).transpose() << "\n";
//        for(int i = 0; i < 3; ++i)
//            std::cout << "  p" << i << "': " << pts.col(i).transpose() << "\n";
//        for(int i = 0; i < 3; ++i)
//            std::cout << "  v" << i << "': " << vs_.col(i).transpose() << "\n";
//        for(int i = 0; i < 3; ++i)
//            std::cout << "  l" << i << ": " << edgeLength(i) << " ("
//                      << vs_.col(i).norm() - edgeLength(i) << ")\n";
//        for(int li = 0; li < 3; ++li)
//        {
//            std::cout << "  dl" << li << ": ";
//            for(int ni = 0; ni < 3; ++ni)
//            {
//                if(ni) std::cout << ", ";
//                std::cout << "dn" << ni << ": " << dldn(li, ni);
//            }
//            std::cout << " (" << dldn(li, 0) + dldn(li, 1) + dldn(li, 2) << ")\n";
//        }

//        for(int i = 0; i < 3; ++i)
//            std::cout << "  b(p" << i << "): " << eval(point(i)).transpose() << "\n";
//        for(int i = 0; i < 3; ++i)
//        {
//            std::cout << "  b(e" << i << "): " << eval((point(i, 1) + point(i, 2)) / 2).transpose() << "\n";
//        }
//        for(int i = 0; i < 3; ++i)
//        {
////            std::cout << "  f: " << _vertexGradientFactor(i, (i+1)%3) << "\n";
////            std::cout << "  f: " << _vertexGradientFactor(i, (i+2)%3) << "\n";
//            std::cout << "  g(e" << i << "): " << _integrateNormalDerivativeOverSegment(
//                             point(i, 1), point(i, 2), 32, .000001).transpose() << "\n";
//        }
//        for(int i = 0; i < 3; ++i)
//        {
//            std::cout << "  g(e" << i << "): " << _integrateNormalDerivativeOverSegment2(
//                             point(i, 1), point(i, 2), 32).transpose() << "\n";
//        }
//        for(int i = 0; i < 3; ++i)
//        {
//            std::cout << "  g(e" << i << "): " << _integrateNormalDerivativeOverSegment2(
//                             point(i, 1), point(i, 2), 256).transpose() << "\n";
//        }
#endif
    }

private:
    using Base::m_points;
    using Base::m_2delta;
    using Base::m_lbf;

    Matrix2 m_rot;

    Vector3 m_eLen;
    Matrix3 m_dldn;
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
    typename Mesh::HalfedgeAroundFaceCirculator hit = mesh.halfedges(element);

    bool orient[3];
    Vector p[3];
    Vector v[3];
    int nodes[9];

// ################# BEGIN TEST CODE
    int flatVx = -1;
    for(int i = 0; i < 3; ++i)
    {
//        if(mesh.vertexGradientConstrained(mesh.toVertex(*hit)))
        if(mesh.toVertex(*hit).idx() == 0)
            flatVx = i;
        ++hit;
    }
    while(flatVx > 0)
    {
        ++hit;
        --flatVx;
    }
//    std::cerr << flatVx << "\n";
// ################# END TEST CODE

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

//    Vector p[] = {
//        Vector::Zero(),
//        v[2],
//        -v[1]
//    };

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

    Vector3 dx2_[9];
    Vector3 dy2_[9];
    Vector3 dxy_[9];
    for(size_t i = 0; i < 3; ++i)
    {
    // 25%:
        dx2_[i+0] = funcVertexBasis(i, Deriv_XX, a, b, c, d, l, area);
        dx2_[i+3] = funcMidpointBasis(i, Deriv_XX, a, b, c, d, l, area);
        dx2_[i+6] = funcMidpointDerivBasis(i, Deriv_XX, a, b, c, d, l, area);
        dy2_[i+0] = funcVertexBasis(i, Deriv_YY, a, b, c, d, l, area);
        dy2_[i+3] = funcMidpointBasis(i, Deriv_YY, a, b, c, d, l, area);
        dy2_[i+6] = funcMidpointDerivBasis(i, Deriv_YY, a, b, c, d, l, area);
        dxy_[i+0] = funcVertexBasis(i, Deriv_XY, a, b, c, d, l, area);
        dxy_[i+3] = funcMidpointBasis(i, Deriv_XY, a, b, c, d, l, area);
        dxy_[i+6] = funcMidpointDerivBasis(i, Deriv_XY, a, b, c, d, l, area);
    };

    typedef Eigen::Matrix<Scalar, 9, 1> Vector9;
    Vector9 b8p0gcons = Vector9::Zero();
    Vector9 b7p0gcons = Vector9::Zero();

    typedef _FVElement<Scalar> Elem;
    Elem elem(p[0], p[1], p[2]);
    {
//        {
//#define FN2(_post) _vertexSubExpr ## _post
//#define FN2(_post) _edgeSubExpr ## _post
//#define FN2(_post) _gradientSubExpr ## _post
//#define FN FN2()
//            Vector ep = .2 * elem.point(0) + .5 * elem.point(1) + .3 * elem.point(2);
//            int bfi = 1;
//            Scalar baseOffset = 1;
//            Vector xoffset(baseOffset / 2, 0);
//            Vector yoffset(0, baseOffset / 2);
//            Vector grad = elem.FN2(Gradient)(bfi, elem.Elem::Base::eval(ep));
//            Scalar rescale = grad.norm();
//            grad /= rescale;
//            Vector prev;
//            for(int step = 0; step < 8; ++step) {
//                Scalar scale = 1 / std::pow(10, step);
//                Vector ng = Vector(
//                    (elem.FN(bfi, elem.Elem::Base::eval(ep + xoffset*scale))
//                            - elem.FN(bfi, elem.Elem::Base::eval(ep - xoffset*scale))),
//                    (elem.FN(bfi, elem.Elem::Base::eval(ep + yoffset*scale))
//                            - elem.FN(bfi, elem.Elem::Base::eval(ep - yoffset*scale)))
//                ) / (baseOffset*scale*rescale) - grad;

//                std::cout << "step " << step << ": " << ng.transpose();
//                if(step)
//                    std::cout << ", " << (prev.array() / ng.array()).transpose();
//                std::cout << "\n";

//                prev = ng;
//            }
//        }

        Scalar l0 = v[0].norm();
        Scalar l1 = v[1].norm();
        Scalar l2 = v[2].norm();

        Eigen::Matrix<Scalar, 2, 2> frame;
        frame.col(0) = v[2] / l2;
        frame(0, 1) = -frame(1, 0);
        frame(1, 1) = frame(0, 0);

//        v[0] = frame.partialPivLu().solve(v[0]);
        Vector p2 = -frame.partialPivLu().solve(v[1]);
//        v[2] = frame.partialPivLu().solve(v[2]);

        Scalar p1x = l2;
        Scalar p2x = p2[0];
        Scalar p2y = p2[1];

//        std::cout << p1x << ", " << p2x << ", " << p2y << "\n";

        Scalar _2delta = p1x * p2y;
//        std::cout << "2 * delta = " << _2delta << "\n";

        //Scalar l2 = v[2].norm();

//        std::cout << l0 << ", " << l1 << ", " << l2 << "\n";

        Scalar dl0dn0 = v[0].dot(v[0]) / (_2delta * l0);
        Scalar dl0dn1 = v[0].dot(v[1]) / (_2delta * l1);
        Scalar dl0dn2 = v[0].dot(v[2]) / (_2delta * l2);

        Scalar dl1dn0 = v[1].dot(v[0]) / (_2delta * l0);
        Scalar dl1dn1 = v[1].dot(v[1]) / (_2delta * l1);
        Scalar dl1dn2 = v[1].dot(v[2]) / (_2delta * l2);

        Scalar dl2dn0 = v[2].dot(v[0]) / (_2delta * l0);
        Scalar dl2dn1 = v[2].dot(v[1]) / (_2delta * l1);
        Scalar dl2dn2 = v[2].dot(v[2]) / (_2delta * l2);

//        std::cout << dl0dn0 << ", " << dl0dn1 << ", " << dl0dn2 << "\n";
//        std::cout << dl1dn0 << ", " << dl1dn1 << ", " << dl1dn2 << "\n";
//        std::cout << dl2dn0 << ", " << dl2dn1 << ", " << dl2dn2 << "\n";

        Scalar a0 = p1x*p2y / _2delta;
        Scalar b1 = p2y / _2delta;
        Scalar b0 = -b1;
        Scalar c0 = (p2x - p1x) / _2delta;
        Scalar c1 = -p2x / _2delta;
        Scalar c2 = p1x / _2delta;

//        std::cout << "a0 = " << a0 << "\n";
//        std::cout << "b0 = " << b0 << "\n";
//        std::cout << "b1 = " << b1 << "\n";
//        std::cout << "c0 = " << c0 << "\n";
//        std::cout << "c1 = " << c1 << "\n";
//        std::cout << "c2 = " << c2 << "\n";

        Scalar hf0 = - _2delta / l0;
        Scalar hf1 = - _2delta / l1;
        Scalar hf2 = - _2delta / l2;

        Scalar vhf_l0_e1 = dl0dn1 + dl1dn1 / 2.;
        Scalar vhf_l0_e2 = dl0dn2 + dl2dn2 / 2.;
        Scalar vhf_l1_e0 = dl1dn0 + dl0dn0 / 2.;
        Scalar vhf_l1_e2 = dl1dn2 + dl2dn2 / 2.;
        Scalar vhf_l2_e0 = dl2dn0 + dl0dn0 / 2.;
        Scalar vhf_l2_e1 = dl2dn1 + dl1dn1 / 2.;

        Scalar tmp0 = pow(b0, 2);
        Scalar tmp1 = 6*a0;
        Scalar tmp2 = pow(b1, 2);
        Scalar tmp3 = 6*hf1;
        Scalar tmp4 = tmp2*tmp3;
        Scalar tmp5 = pow(b0, 3);
        Scalar tmp6 = pow(b1, 3);
        Scalar tmp7 = 12*hf1;
        Scalar tmp8 = tmp6*tmp7;
        Scalar tmp9 = b0*b1*c2;
        Scalar tmp10 = 6*tmp9;
        Scalar tmp11 = c0*tmp0;
        Scalar tmp12 = c1*tmp2;
        Scalar tmp13 = tmp12*tmp7;
        Scalar tmp14 = tmp10 + 6*tmp11 + tmp13*vhf_l0_e1;
        Scalar tmp15 = b0*c0;
        Scalar tmp16 = a0*b1*c2;
        Scalar tmp17 = 3*tmp16;
        Scalar tmp18 = b1*c1;
        Scalar tmp19 = tmp18*tmp3;
        Scalar tmp20 = b0*c1*c2;
        Scalar tmp21 = b1*c0*c2;
        Scalar tmp22 = 6*tmp20 + 6*tmp21;
        Scalar tmp23 = pow(c0, 2);
        Scalar tmp24 = b0*tmp23;
        Scalar tmp25 = pow(c1, 2);
        Scalar tmp26 = b1*tmp25;
        Scalar tmp27 = tmp26*tmp7;
        Scalar tmp28 = tmp22 + 6*tmp24 + tmp27*vhf_l0_e1;
        Scalar tmp29 = c1*c2;
        Scalar tmp30 = tmp1*tmp29;
        Scalar tmp31 = tmp25*tmp3;
        Scalar tmp32 = pow(c2, 2);
        Scalar tmp33 = 6*hf2*tmp32;
        Scalar tmp34 = pow(c0, 3);
        Scalar tmp35 = c0*c1*c2;
        Scalar tmp36 = 18*tmp35;
        Scalar tmp37 = pow(c1, 3);
        Scalar tmp38 = tmp37*tmp7;
        Scalar tmp39 = pow(c2, 3);
        Scalar tmp40 = 12*hf2*tmp39;
        Scalar tmp41 = 6*hf0*tmp0;
        Scalar tmp42 = 12*a0*hf0*tmp0;
        Scalar tmp43 = 12*hf0;
        Scalar tmp44 = tmp43*tmp5;
        Scalar tmp45 = tmp11*tmp43;
        Scalar tmp46 = tmp10 + 6*tmp12 + tmp45*vhf_l1_e0;
        Scalar tmp47 = 6*b0*c0*hf0;
        Scalar tmp48 = 12*a0*b0*c0*hf0;
        Scalar tmp49 = tmp24*tmp43;
        Scalar tmp50 = tmp22 + 6*tmp26 + tmp49*vhf_l1_e0;
        Scalar tmp51 = 6*hf0*tmp23;
        Scalar tmp52 = 12*a0*hf0*tmp23;
        Scalar tmp53 = tmp34*tmp43;
        Scalar tmp54 = tmp10 + tmp13*vhf_l2_e1 + tmp45*vhf_l2_e0;
        Scalar tmp55 = tmp22 + tmp27*vhf_l2_e1 + tmp49*vhf_l2_e0;
        Scalar tmp56 = 48*a0;
        Scalar tmp57 = b1*c2;
        Scalar tmp58 = 24*b0*(2*tmp15 - tmp57);
        Scalar tmp59 = -12*tmp16;
        Scalar tmp60 = -24*tmp20 - 24*tmp21;
        Scalar tmp61 = 48*tmp24 + tmp60;
        Scalar tmp62 = -24*a0*tmp29;
        Scalar tmp63 = -72*tmp35;
        Scalar tmp64 = b0*c2;
        Scalar tmp65 = 24*b1*(2*tmp18 - tmp64);
        Scalar tmp66 = 48*tmp26 + tmp60;
        Scalar tmp67 = 8*c0;
        Scalar tmp68 = -24*tmp9;
        Scalar tmp69 = b0*c1;
        Scalar tmp70 = b1*c0;
        Scalar tmp71 = -24*c2*(tmp69 + tmp70);
        Scalar tmp72 = 2*a0 - 1;

        dx2[0] = Vector3(tmp0*tmp1 + tmp0 - tmp4*vhf_l0_e1, 6*tmp5 + tmp8*vhf_l0_e1, tmp14);
        dx2[1] = Vector3(tmp2 - tmp41*vhf_l1_e0 + tmp42*vhf_l1_e0, tmp44*vhf_l1_e0 + 6*tmp6, tmp46);
        dx2[2] = Vector3(-tmp4*vhf_l2_e1 - tmp41*vhf_l2_e0 + tmp42*vhf_l2_e0, tmp44*vhf_l2_e0 + tmp8*vhf_l2_e1, tmp54);
        dx2[3] = Vector3(tmp0*(tmp56 - 24), 48*tmp5, tmp58);
        dx2[4] = Vector3(-24*tmp2, 48*tmp6, tmp65);
        dx2[5] = Vector3(8*b0*b1, 0, tmp68);
        dx2[6] = Vector3(tmp41*tmp72, tmp44, tmp45);
        dx2[7] = Vector3(-tmp4, tmp8, tmp13);
        dx2[8] = Vector3(0, 0, 0);
        dxy[0] = Vector3(tmp1*tmp15 + tmp15 + tmp17 - tmp19*vhf_l0_e1, tmp14, tmp28);
        dxy[1] = Vector3(tmp17 + tmp18 - tmp47*vhf_l1_e0 + tmp48*vhf_l1_e0, tmp46, tmp50);
        dxy[2] = Vector3(tmp17 - tmp19*vhf_l2_e1 - tmp47*vhf_l2_e0 + tmp48*vhf_l2_e0, tmp54, tmp55);
        dxy[3] = Vector3(tmp15*tmp56 - 24*tmp15 + 4*tmp57 + tmp59, tmp58, tmp61);
        dxy[4] = Vector3(-24*tmp18 + tmp59 + 4*tmp64, tmp65, tmp66);
        dxy[5] = Vector3(tmp59 + 4*tmp69 + 4*tmp70, tmp68, tmp71);
        dxy[6] = Vector3(tmp47*tmp72, tmp45, tmp49);
        dxy[7] = Vector3(-tmp19, tmp13, tmp27);
        dxy[8] = Vector3(0, 0, 0);
        dy2[0] = Vector3(tmp1*tmp23 + tmp23 + tmp30 - tmp31*vhf_l0_e1 - tmp33*vhf_l0_e2, tmp28, 6*tmp34 + tmp36 + tmp38*vhf_l0_e1 + tmp40*vhf_l0_e2);
        dy2[1] = Vector3(tmp25 + tmp30 - tmp33*vhf_l1_e2 - tmp51*vhf_l1_e0 + tmp52*vhf_l1_e0, tmp50, tmp36 + 6*tmp37 + tmp40*vhf_l1_e2 + tmp53*vhf_l1_e0);
        dy2[2] = Vector3(tmp30 - tmp31*vhf_l2_e1 + tmp32 - tmp51*vhf_l2_e0 + tmp52*vhf_l2_e0, tmp55, tmp36 + tmp38*vhf_l2_e1 + 6*tmp39 + tmp53*vhf_l2_e0);
        dy2[3] = Vector3(tmp23*tmp56 - 24*tmp23 + 8*tmp29 + tmp62, tmp61, 48*tmp34 + tmp63);
        dy2[4] = Vector3(c2*tmp67 - 24*tmp25 + tmp62, tmp66, 48*tmp37 + tmp63);
        dy2[5] = Vector3(c1*tmp67 - 24*tmp32 + tmp62, tmp71, 48*tmp39 + tmp63);
        dy2[6] = Vector3(tmp51*tmp72, tmp49, tmp53);
        dy2[7] = Vector3(-tmp31, tmp27, tmp38);
        dy2[8] = Vector3(-tmp33, 0, tmp40);

//        std::cout << "Element:\n";
//        for(int i = 0; i < 9; ++i)
//        {
//            for(int j = 0; j < 3; ++j)
//            {
//                if(abs(dx2[i][j] - dx22[i][j]) > 1.e-4)
//                    std::cout << "b" << i << "xx" << j << " = "
//                              << dx2[i][j] << " | " << dx22[i][j] << " | " << dx2[i][j]/dx22[i][j] << "\n";
//                if(abs(dxy[i][j] - dxy2[i][j]) > 1.e-4)
//                    std::cout << "b" << i << "xy" << j << " = "
//                              << dxy[i][j] << " | " << dxy2[i][j] << " | " << dxy[i][j]/dxy2[i][j] << "\n";
//                if(abs(dy2[i][j] - dy22[i][j]) > 1.e-4)
//                    std::cout << "b" << i << "yy" << j << " = "
//                              << dy2[i][j] << " | " << dy22[i][j] << " | " << dy2[i][j]/dy22[i][j] << "\n";
//            }
    //        std::cout << dx2[i].format(fmt) << "\n"
    //                  << dxy[i].format(fmt) << "\n"
    //                  << dy2[i].format(fmt) << "\n";
//        }

//        if(flatVx == 0)
//        {
//            b8p0gcons <<
//                _2delta*dl0dn2/(l1*l2) + 4/l1,
//                -_2delta*dl1dn0/(l0*l1) + _2delta*dl1dn2/(l1*l2),
//                -_2delta*dl2dn0/(l0*l1),
//                4/l1,
//                -4/l1,
//                -4/l1,
//                -_2delta/(l0*l1),
//                0,
//                _2delta/(l1*l2);
//            b7p0gcons <<
//                -_2delta*dl0dn1/(l1*l2) - 4/l2,
//                _2delta*dl1dn0/(l0*l2),
//                _2delta*dl2dn0/(l0*l2) - _2delta*dl2dn1/(l1*l2),
//                -4/l2,
//                4/l2,
//                4/l2,
//                _2delta/(l0*l2),
//                -_2delta/(l1*l2),
//                0;
//        }
    }

//    std::cout << "Element:\n";
//    Eigen::IOFormat fmt(4, 0, ", ", "\n", "    ", "", "", "");
//    for(int i = 0; i < 9; ++i) {
//        Eigen::Matrix<Scalar, 3, 3> m;
//        m << dx2_[i], dx2[i], dx2[i] - dx2_[i];
//        std::cout << "  dx2[" << i << "]:\n" << m.format(fmt) << "\n";
//    }
//    for(int i = 0; i < 9; ++i) {
//        Eigen::Matrix<Scalar, 3, 3> m;
//        m << dy2_[i], dy2[i], dy2[i] - dy2_[i];
//        std::cout << "  dy2[" << i << "]:\n" << m.format(fmt) << "\n";
//    }
//    for(int i = 0; i < 9; ++i) {
//        Eigen::Matrix<Scalar, 3, 3> m;
//        m << dxy_[i], dxy[i], dxy[i] - dxy_[i];
//        std::cout << "  dxy[" << i << "]:\n" << m.format(fmt) << "\n";
//    }

//    for(size_t i = 0; i < 9; ++i)
//    {
//        dx2[i] = dx2_[i];
//        dy2[i] = dy2_[i];
//        dxy[i] = dxy_[i];
//    }

    Vector3 quad_bc[3] = {
        Vector3(0, .5, .5),
        Vector3(.5, 0, .5),
        Vector3(.5, .5, 0)
    };

    typedef Eigen::Matrix<Scalar, 9, 1> Vector9;
    Vector9 eval_dx2[3];
    Vector9 eval_dy2[3];
    Vector9 eval_dxy[3];
    typename Elem::Matrix2 hessian[9];
    for(int pi = 0; pi < 3; ++pi)
    {
        elem.hessian(quad_bc[pi], hessian);
        for(int bi = 0; bi < 9; ++bi)
        {
            eval_dx2[pi](bi) = hessian[bi](0, 0);
            eval_dy2[pi](bi) = hessian[bi](1, 1);
            eval_dxy[pi](bi) = hessian[bi](0, 1);
        }
    }

    Eigen::Matrix<Scalar, 9, 9> m;
    for(size_t i = 0; i < 9; ++i)
    {
        for(size_t j = i; j < 9; ++j)
        {
            Scalar value = 0;
            EIGEN_ASM_COMMENT("MYBEGIN");

            Vector6 basis = multBasis(dx2[i]+dy2[i], dx2[j]+dy2[j])
                                    + (1.-m_sigma) * (2. * multBasis(dxy[i], dxy[j])
                                                    - multBasis(dx2[i], dy2[j])
                                                    - multBasis(dy2[i], dx2[j]));

            value = integrateQuadTriangle(v, basis, area);

            Scalar value2 = 0;
            for(int pi = 0; pi < 3; ++pi)
            {
                value2 += elem.doubleArea()/6 * (
                    (eval_dx2[pi](i) + eval_dy2[pi](i)) * (eval_dx2[pi](j) + eval_dy2[pi](j))
                        + (1.-m_sigma) * (2. * eval_dxy[pi](i) * eval_dxy[pi](j)
                                          - eval_dx2[pi](i) * eval_dy2[pi](j)
                                          - eval_dy2[pi](i) * eval_dx2[pi](j)));
//                value2 += area * eval_vecs2[pi].dot(basis) / 3;
            }
//            std::cout << "Test int: " << value << ", " << value2 << ", "
//                      << value2 - value << ", " << value2 / value << "\n";
            value = value2;

            EIGEN_ASM_COMMENT("MYEND");

            if((i < 6 || orient[i%3]) != (j < 6 || orient[j%3]))
            {
                value *= -1;
            }

//            *(it++) = Triplet(nodes[i], nodes[j], value);
//            if(i != j)
//                *(it++) = Triplet(nodes[j], nodes[i], value);

            m(i, j) = value;
            m(j, i) = value;
        }
    }

    if(flatVx == 0)
    {
//        std::cout << "Point Gradient Constraint 2\n";
//        Scalar _2delta = elem.doubleArea();
//        Scalar dldn[3][3];
//        Scalar el[3];

//        Vector9 fde1;
//        Vector9 fde2;


//        m.row(7) = fde2;
//        m.row(8) = fde1;
//        m.col(7) = fde2;
//        m.col(8) = fde1;

//        m.row(7) = b7p0gcons;
//        m.row(8) = b8p0gcons;
//        m.col(7) = b7p0gcons;
//        m.col(8) = b8p0gcons;

//        m.row(8) = b7p0gcons;
//        m.row(7) = b8p0gcons;
//        m.col(8) = b7p0gcons;
//        m.col(7) = b8p0gcons;
//        std::cerr << m << "\n";
    }

    for(size_t i = 0; i < 9; ++i)
    {
        for(size_t j = 0; j < 9; ++j)
        {
            *(it++) = Triplet(nodes[i], nodes[j], m(i, j));
        }
    }
}

//template < class _Mesh, typename _Scalar >
//template < typename InIt >
//void
//FVElementBuilder<_Mesh, _Scalar>::
//    processFV1ElementFlat(InIt& it, const Mesh& mesh, Face element) const
//{
//    typename Mesh::HalfedgeAroundFaceCirculator hit = mesh.halfedges(element);

//    // FIXME: This implementation assume that the element is equilateral.

//    bool orient[3];
//    Vector v[3];
//    unsigned nodes[9];
//    --hit;
//    unsigned flatVertex;
//    for(int i = 0; i < 3; ++i)
//    {
//        v[i] = (mesh.position(mesh.toVertex(*hit)) -
//                mesh.position(mesh.fromVertex(*hit))).template cast<Scalar>();
//        orient[i] = mesh.fromVertex(*hit).idx() > mesh.toVertex(*hit).idx();
//        nodes[3+i] = mesh.edgeValueNode(*hit).idx();
//        nodes[6+i] = mesh.gradientNode(*hit);
//        ++hit;
//        nodes[i] = mesh.toNode(*hit);
//        if(mesh.hasFlatGradient(mesh.toVertex(*hit)))
//            flatVertex = i;
//    }

//    Scalar area = det2(v[0], v[1]) / 2.;
//    assert(area > 0);

//    Scalar sqrtArea = std::sqrt(area);
//    Scalar ffd = (14.-12.*m_sigma)/(3.*area);
//    Scalar ffo = (5.-6.*m_sigma)/(3.*area);
//    Scalar fgd = (12.*m_sigma+4.)/(3.*area);
//    Scalar fgo = (6.*m_sigma-14.)/(3.*area);
//    Scalar fhd = -1.754765350603323/sqrtArea;
//    Scalar fho = (0.43869133765083*(6.*m_sigma-4.))/sqrtArea;
//    Scalar ggd = (24.*m_sigma+104.)/(3.*area);
//    Scalar ggo = -(24.*m_sigma+40.)/(3.*area);
//    Scalar ghd = -(5.264296051809969*m_sigma+8.773826753016614)/sqrtArea;
//    Scalar gho = 7.019061402413293/sqrtArea;
//    Scalar hhd = 6.928203230275509;
//    Scalar hho = 0.;

//    typedef Eigen::Matrix<Scalar, 9, 9> StiffnessMatrix;
//    StiffnessMatrix elemStiffness;
//    elemStiffness <<
//        ffd, ffo, ffo,	fgd, fgo, fgo,	fhd, fho, fho,
//        ffo, ffd, ffo,	fgo, fgd, fgo,	fho, fhd, fho,
//        ffo, ffo, ffd,	fgo, fgo, fgd,	fho, fho, fhd,

//        fgd, fgo, fgo,	ggd, ggo, ggo,	ghd, gho, gho,
//        fgo, fgd, fgo,	ggo, ggd, ggo,	gho, ghd, gho,
//        fgo, fgo, fgd,	ggo, ggo, ggd,	gho, gho, ghd,

//        fhd, fho, fho,	ghd, gho, gho,	hhd, hho, hho,
//        fho, fhd, fho,	gho, ghd, gho,	hho, hhd, hho,
//        fho, fho, fhd, 	gho, gho, ghd,	hho, hho, hhd;

//    typedef Eigen::Matrix<Scalar, 9, 1> Vector9;

//    static const Vector9 u8_1((Vector9()
//            <<  -2.659424899780574/sqrtArea, -0.3799178428258/sqrtArea,                        0.,
//                 -3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea,
//                                         1.,                       -1.,                        0.).finished());
//    static const Vector9 u9_1((Vector9()
//            << -2.659424899780574/sqrtArea,                        0., -0.3799178428258/sqrtArea,
//                -3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea,
//                                        1.,                        0.,                       -1.).finished());
//    static const Vector9 u7_2((Vector9()
//            << -0.3799178428258/sqrtArea, -2.659424899780574/sqrtArea,                        0.,
//               3.03934274260637/sqrtArea,  -3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea,
//                                     -1.,                          1.,                        0.).finished());
//    static const Vector9 u9_2((Vector9()
//            <<                        0., -2.659424899780574/sqrtArea, -0.3799178428258/sqrtArea,
//               3.03934274260637/sqrtArea,  -3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea,
//                                      0.,                          1.,                       -1.).finished());
//    static const Vector9 u7_3((Vector9()
//            << -0.3799178428258/sqrtArea,                        0., -2.659424899780574/sqrtArea,
//               3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea,  -3.03934274260637/sqrtArea,
//                                     -1.,                         0,                          1.).finished());
//    static const Vector9 u8_3((Vector9()
//            <<                        0., -0.3799178428258/sqrtArea, -2.659424899780574/sqrtArea,
//               3.03934274260637/sqrtArea, 3.03934274260637/sqrtArea,  -3.03934274260637/sqrtArea,
//                                      0.,                       -1.,                          1.).finished());

//    if(flatVertex == 0)
//    {
//        elemStiffness.row(7) = u8_1;
//        elemStiffness.col(7) = u8_1;
//        elemStiffness.row(8) = u9_1;
//        elemStiffness.col(8) = u9_1;
//    }
//    else if(flatVertex == 1)
//    {
//        elemStiffness.row(6) = u7_2;
//        elemStiffness.col(6) = u7_2;
//        elemStiffness.row(8) = u9_2;
//        elemStiffness.col(8) = u9_2;
//    }
//    else
//    {
//        elemStiffness.row(6) = u7_3;
//        elemStiffness.col(6) = u7_3;
//        elemStiffness.row(7) = u8_3;
//        elemStiffness.col(7) = u8_3;
//    }

//    // Flip gradient sign where needed.
//    for(size_t i = 0; i < 9; ++i)
//    {
//        for(size_t j = i; j < 9; ++j)
//        {
//            Scalar sign = 1;
//            if((i < 6 || orient[i % 3]) != (j < 6 || orient[j % 3]))
//            {
//                sign = -1;
//            }

//            *(it++) = Triplet(nodes[i], nodes[j], elemStiffness(i, j) * sign);
//            if(i != j)
//                *(it++) = Triplet(nodes[j], nodes[i], elemStiffness(j, i) * sign);
//        }
//    }
//}

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
