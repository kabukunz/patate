/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


#include <Eigen/Dense>

#include "fvElement.h"

#include "fvElementBuilder.h"


namespace Vitelotte
{


template < typename _Scalar >
class FVElementFlat : public FVElement<_Scalar>
{
public:
    typedef _Scalar Scalar;
    typedef FVElement<Scalar> Base;

    typedef Eigen::Matrix<Scalar, 2, 1> Vector;
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<Scalar, 9, 1> Vector9;

    typedef Eigen::Matrix<Scalar, 2, 2> Matrix2;
    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

    typedef Eigen::Matrix<Scalar, 2, 3> Matrix2x3;
    typedef Eigen::Matrix<Scalar, 9, 2> Matrix9x2;


public:
    inline FVElementFlat(const Vector& p0, const Vector& p1, const Vector& p2)
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
    nCoefficients(const Mesh& mesh, Face element) const
{
    typename Mesh::template VertexProperty<bool> isGc =
            mesh.template getVertexProperty<bool>("v:isGradientConstraint");

    typename Mesh::VertexAroundFaceCirculator vit = mesh.vertices(element);
    typename Mesh::VertexAroundFaceCirculator vend = vit;
    do ++vit;
    while(!isGc[*vit] && vit != vend);
    bool isPgc = isGc[*vit];

    if(isPgc) std::cout << "pgc1: " << element.idx() << "\n";

    return isPgc? 117: 81;
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

    processFV1Element(it, mesh, element);
}

template < class _Mesh, typename _Scalar >
template < typename InIt >
void
FVElementBuilder<_Mesh, _Scalar>::
    processFV1Element(InIt& it, const Mesh& mesh, Face element)
{
    typedef Eigen::Matrix<Scalar, 9, 9> Matrix9;
    Matrix9 sm;

    int nodes[9];

    typename Mesh::template VertexProperty<bool> isGc =
            mesh.template getVertexProperty<bool>("v:isGradientConstraint");

    typename Mesh::HalfedgeAroundFaceCirculator hit = mesh.halfedges(element);
    typename Mesh::HalfedgeAroundFaceCirculator hend = hit;
    do ++hit;
    while(!isGc[mesh.toVertex(*hit)] && hit != hend);
    bool isPgc = isGc[mesh.toVertex(*hit)];
    typename Mesh::Edge e2 = mesh.edge(*hit);
    ++hit;
    typename Mesh::Edge e1 = mesh.edge(*hit);
    --hit;
    if(isPgc) std::cout << "pgc2: " << element.idx() << "\n";

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

    typedef FVElement<Scalar> Elem;
    Elem elem(p);

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
//        if(!flat)
//        {
            elem.hessian(bc, hessians);
//        }
//        else
//        {
//            typedef FVElementFlat<Scalar> ElemFlat;
//            ElemFlat elemFlat(p[0], p[1], p[2]);
//            elemFlat.hessian(bc, hessians);
//            orient[7] = 1;
//            orient[8] = 1;

//            Scalar delta = 1.e-6;

//            Vector dx(delta/2, 0);
//            Vector dy(0, delta/2);
//            for(unsigned bi = 0; bi < 9; ++bi)
//            {
//                hessians[bi](0, 0) = elemFlat.eval(elemFlat.point(pi) + dx + dx)(bi)
//                                   - elemFlat.eval(elemFlat.point(pi) + dx - dx)(bi)
//                                   - elemFlat.eval(elemFlat.point(pi) - dx + dx)(bi)
//                                   + elemFlat.eval(elemFlat.point(pi) - dx - dx)(bi);
//                hessians[bi](0, 1) = elemFlat.eval(elemFlat.point(pi) + dx + dy)(bi)
//                                   - elemFlat.eval(elemFlat.point(pi) + dx - dy)(bi)
//                                   - elemFlat.eval(elemFlat.point(pi) - dx + dy)(bi)
//                                   + elemFlat.eval(elemFlat.point(pi) - dx - dy)(bi);
//                hessians[bi](1, 0) = hessians[bi](0, 1);
//                hessians[bi](1, 1) = elemFlat.eval(elemFlat.point(pi) + dy + dy)(bi)
//                                   - elemFlat.eval(elemFlat.point(pi) + dy - dy)(bi)
//                                   - elemFlat.eval(elemFlat.point(pi) - dy + dy)(bi)
//                                   + elemFlat.eval(elemFlat.point(pi) - dy - dy)(bi);
//                hessians[bi] /= delta * delta;
//            }

//            typedef Eigen::Matrix<Scalar, 9, 2> Matrix9x2;
//            Vector dv1 = (elemFlat.point(1) - elemFlat.point(0)).normalized() * (delta/2);
//            Vector dv2 = (elemFlat.point(2) - elemFlat.point(0)).normalized() * (delta/2);
//            Matrix9x2 testDiffP0;
//            testDiffP0 <<
//                (elemFlat.eval(elemFlat.point(0) + dv2) - elemFlat.eval(elemFlat.point(0) - dv2)) / delta,
//                (elemFlat.eval(elemFlat.point(0) + dv1) - elemFlat.eval(elemFlat.point(0) - dv1)) / delta;
//            std::cout << "Test diff p0:\n" << testDiffP0.transpose() << "\n";
//        }

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
            sm(i, j) = value;
            sm(j, i) = value;
        }
    }

    for(size_t i = 0; i < 9; ++i)
    {
        for(size_t j = 0; j < 9; ++j)
        {
            *(it++) = Triplet(nodes[i], nodes[j], sm(i, j));
        }
    }
    if(isPgc)
    {
        std::cout << "Flat elem:\n";
        std::cout << "  p0: " << elem.point(0).transpose() << "\n";
        std::cout << "  p1: " << elem.point(1).transpose() << "\n";
        std::cout << "  p2: " << elem.point(2).transpose() << "\n";
        std::cout << "  n8: " << mesh.nodeValue(typename Mesh::Node(nodes[8])).transpose() << "\n";
        std::cout << "  n7: " << mesh.nodeValue(typename Mesh::Node(nodes[7])).transpose() << "\n";
        std::cout << "  Stiffness matrix:\n" << sm << "\n";

        typedef Eigen::Matrix<Scalar, 9, 1> Vector9;
        Vector9 fde1, fde2;
        fde1 <<
            -1.0L/2.0L*(elem.doubleArea()*(2*elem.dldn(0, 1) + elem.dldn(1, 1)) + 7*elem.edgeLength(1))/(elem.edgeLength(1)*elem.edgeLength(2)),
            (1.0L/2.0L)*(elem.doubleArea()*(elem.dldn(0, 0) + 2*elem.dldn(1, 0)) - elem.edgeLength(0))/(elem.edgeLength(0)*elem.edgeLength(2)),
            -1.0L/2.0L*elem.doubleArea()*(elem.edgeLength(0)*(elem.dldn(1, 1) + 2*elem.dldn(2, 1)) - elem.edgeLength(1)*(elem.dldn(0, 0) + 2*elem.dldn(2, 0)))/(elem.edgeLength(0)*elem.edgeLength(1)*elem.edgeLength(2)),
            -4/elem.edgeLength(2),
            4/elem.edgeLength(2),
            4/elem.edgeLength(2),
            elem.doubleArea()/(elem.edgeLength(0)*elem.edgeLength(2)),
            -elem.doubleArea()/(elem.edgeLength(1)*elem.edgeLength(2)),
            0;
        fde2 <<
            -1.0L/2.0L*(elem.doubleArea()*(2*elem.dldn(0, 2) + elem.dldn(2, 2)) + 7*elem.edgeLength(2))/(elem.edgeLength(1)*elem.edgeLength(2)),
            -1.0L/2.0L*elem.doubleArea()*(elem.edgeLength(0)*(2*elem.dldn(1, 2) + elem.dldn(2, 2)) - elem.edgeLength(2)*(elem.dldn(0, 0) + 2*elem.dldn(1, 0)))/(elem.edgeLength(0)*elem.edgeLength(1)*elem.edgeLength(2)),
            (1.0L/2.0L)*(elem.doubleArea()*(elem.dldn(0, 0) + 2*elem.dldn(2, 0)) - elem.edgeLength(0))/(elem.edgeLength(0)*elem.edgeLength(1)),
            -4/elem.edgeLength(1),
            4/elem.edgeLength(1),
            4/elem.edgeLength(1),
            elem.doubleArea()/(elem.edgeLength(0)*elem.edgeLength(1)),
            0,
            -elem.doubleArea()/(elem.edgeLength(1)*elem.edgeLength(2));

        typename Mesh::template EdgeProperty<typename Mesh::Node> pgcNode =
                mesh.template getEdgeProperty<typename Mesh::Node>("e:pgcNode");
        int ce1 = pgcNode[e1].idx();
        int ce2 = pgcNode[e2].idx();
        if(ce1 < 0 || ce2 < 0)
        {
            error(STATUS_ERROR, "Invalid node");
            return;
        }
        std::cout << "  ce1: " << e1 << ", " << ce1 << ", "
                  << mesh.nodeValue(pgcNode[e1]).transpose() << "\n";
        std::cout << "  ce2: " << e2 << ", " << ce2 << ", "
                  << mesh.nodeValue(pgcNode[e2]).transpose() << "\n";
        for(size_t i = 0; i < 9; ++i)
        {
            *(it++) = Triplet(nodes[i], ce1, fde1(i));
            *(it++) = Triplet(ce1, nodes[i], fde1(i));
            *(it++) = Triplet(nodes[i], ce2, fde2(i));
            *(it++) = Triplet(ce2, nodes[i], fde2(i));
        }
    }
}


}
