/*
 Copyright (C) 2014

 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


/*!
 \file test/Grenaille/fit_plane.cpp
 \brief Test validity of element classes
 */

#include <vector>

#include <Patate/vitelotte.h>

#include "../common/testing.h"
#include "../common/testUtils.h"

using namespace std;
using namespace Vitelotte;


template<typename Point>
struct PointToProj {
    typedef typename Point::Scalar Scalar;
    typedef Eigen::Matrix<Scalar, 2, Point::SizeAtCompileTime> PTPM;
    typedef Eigen::Matrix<Scalar, 2, 1> Vector2;

    PointToProj(const Point* pts)
        : m_p0(pts[0]),
          m_ptpm(2, m_p0.size())
    {
        m_ptpm.row(0) = (pts[1] - m_p0).normalized();
        Point v2 = (pts[2] - m_p0);
        Scalar p2x = v2.dot(m_ptpm.row(0));
        m_ptpm.row(1) = (v2.transpose() - p2x * m_ptpm.row(0)).normalized();
    }
    Vector2 operator()(const Point& v) const
    {
        return m_ptpm * (v - m_p0);
    }

private:
    Point m_p0;
    PTPM m_ptpm;
};


template<typename Vector>
void randomPoints(Vector* pts, unsigned count,
                  unsigned dims = Vector::SizeAtCompileTime)
{
    for(unsigned i = 0; i < count; ++i)
    {
        pts[i] = Vector::Random(dims);
    }
}


template<typename Elem>
typename Elem::Values evalAll(const Elem& elem,
                              const typename Elem::BarycentricCoord& bc)
{
    typedef typename Elem::Values Values;
    Values values;
    for(int i = 0; i < Values::RowsAtCompileTime; ++i)
    {
        values(i) = elem.eval(i, bc);
    }
    return values;
}


template<typename Elem>
typename Elem::Jacobian gradientAll(const Elem& elem,
                                    const typename Elem::BarycentricCoord& bc)
{
    typedef typename Elem::Jacobian Jacobian;
    Jacobian jacobian;
    for(int i = 0; i < Jacobian::RowsAtCompileTime; ++i)
    {
        jacobian.row(i) = elem.gradient(i, bc);
    }
    return jacobian;
}


template<typename Elem, typename Vector, typename Scalar>
typename Elem::Jacobian numJacobian(const Elem& elem,
                                    const Vector& p,
                                    Scalar epsilon)
{
    typedef typename Elem::Jacobian Jacobian;
    Jacobian jacobian;
    Vector dx(epsilon / 2, 0);
    jacobian.col(0) = (  elem.eval(elem.bcProj(p + dx))
                       - elem.eval(elem.bcProj(p - dx))) / epsilon;
    Vector dy(0, epsilon / 2);
    jacobian.col(1) = (  elem.eval(elem.bcProj(p + dy))
                       - elem.eval(elem.bcProj(p - dy))) / epsilon;
    return jacobian;
}


template<typename Elem, typename Vector, typename Scalar>
typename Elem::Hessian numHessian(const Elem& elem, unsigned bi,
                                  const Vector& p, Scalar epsilon)
{
    typedef typename Elem::BarycentricCoord Bary;
    typedef typename Elem::Hessian Hessian;
    Hessian hessian;
    Vector dv[] = {
        Vector(epsilon / 2, 0),
        Vector(0, epsilon / 2)
    };
    for(int d1 = 0; d1 < 2; ++d1)
    {
        for(int d2 = 0; d2 < 2; ++d2)
        {
            Bary bcp = elem.bcProj(p + dv[d2]);
            Bary bcm = elem.bcProj(p - dv[d2]);
            hessian(d1, d2) = (elem.gradient(bi, bcp)(d1) - elem.gradient(bi, bcm)(d1)) / epsilon;
        }
    }
    return hessian;
}


template<typename Point, typename Elem>
void testElement(const Point* pts, const Point& p) {
    typedef typename Elem::Scalar Scalar;
    typedef typename Elem::BarycentricCoord Bary;

    Scalar epsilon = testEpsilon<Scalar>();

    PointToProj<Point> ptp(pts);
    Elem elem(pts);
    Bary bc = elem.bcProj(ptp(p));

    VERIFY(evalAll(elem, bc).isApprox(elem.eval(bc), epsilon));
    VERIFY(numJacobian(elem, ptp(p), epsilon).isApprox(elem.jacobian(bc), epsilon));
    VERIFY(gradientAll(elem, bc).isApprox(elem.jacobian(bc), epsilon));
}


template<typename Point, typename Elem>
void testElementHessian(const Point* pts, const Point& p) {
    typedef typename Elem::Scalar Scalar;
    typedef typename Elem::BarycentricCoord Bary;
    typedef typename Elem::Hessian Hessian;

    enum
    {
        Size = Elem::Values::RowsAtCompileTime
    };

    Scalar epsilon = testEpsilon<Scalar>();

    PointToProj<Point> ptp(pts);
    Elem elem(pts);
    Bary bc = elem.bcProj(ptp(p));

    Hessian hessians[Size];
    elem.hessian(bc, hessians);
    for(unsigned i = 0; i < Size; ++i)
    {
        Hessian h = elem.hessian(i, bc);
        VERIFY(h.isApprox(hessians[i], epsilon));
        VERIFY(numHessian(elem, i, ptp(p), epsilon).isApprox(h, epsilon));
    }
}


template<typename Point>
void testLinearElement(const Point* pts) {
    typedef typename Point::Scalar Scalar;
    typedef LinearElement<Scalar> Elem;
    typedef typename Elem::BarycentricCoord Bary;
    typedef typename Elem::Vector Vector;

    Scalar epsilon = testEpsilon<Scalar>();

    PointToProj<Point> ptp(pts);
    Elem elem(pts);
    Vector p = Vector::Random();
    Bary bc = elem.bcProj(p);

    VERIFY(evalAll(elem, bc).isApprox(bc, epsilon));
    for(unsigned i = 0; i < 3; ++i)
    {
        VERIFY(elem.projPoint(i).isApprox(ptp(pts[i]), epsilon));
        Bary bc(i==0, i==1, i==2);
        Scalar eProjLen = (elem.projPoint(i, 2) - elem.projPoint(i, 1)).squaredNorm();
        Scalar eRealLen = (pts[(i+2)%3] - pts[(i+1)%3]).squaredNorm();
        VERIFY(std::abs(eProjLen - eRealLen) < epsilon);
        VERIFY(std::abs(elem.edgeLength(i) - std::sqrt(eRealLen)) < epsilon);
        VERIFY(elem.bcProj(ptp(pts[i])).isApprox(bc, epsilon));
        VERIFY(elem.eval(bc).isApprox(bc, epsilon));
    }
}


template<typename Point>
void testQuadraticElement(const Point* pts) {
    typedef typename Point::Scalar Scalar;
    typedef QuadraticElement<Scalar> Elem;
    typedef typename Elem::BarycentricCoord Bary;
    typedef typename Elem::Values Values;

    Scalar epsilon = testEpsilon<Scalar>();

    PointToProj<Point> ptp(pts);
    Elem elem(pts);
    for(unsigned i = 0; i < 3; ++i)
    {
        Bary vbc(i==0, i==1, i==2);
        Bary mbc = Bary(i!=0, i!=1, i!=2) / 2;
        Values vVert, vEdge;
        for(unsigned j = 0; j < 6; ++j)
        {
            vVert(j) = (j == i);
            vEdge(j) = (j == i+3);
        }

        VERIFY(elem.projPoint(i).isApprox(ptp(pts[i]), epsilon));
        VERIFY(elem.bcProj(ptp(pts[i])).isApprox(vbc, epsilon));

        VERIFY(elem.eval(vbc).isApprox(vVert, epsilon));
        VERIFY(elem.eval(mbc).isApprox(vEdge, epsilon));
    }
}


template<typename Point>
void testMorleyElement(const Point* pts) {
    typedef typename Point::Scalar Scalar;
    typedef MorleyElement<Scalar> Elem;
    typedef typename Elem::BarycentricCoord Bary;
    typedef typename Elem::Vector Vector;
    typedef typename Elem::Values Values;

    Scalar epsilon = testEpsilon<Scalar>();

    PointToProj<Point> ptp(pts);
    Elem elem(pts);
    for(unsigned i = 0; i < 3; ++i)
    {
        Bary vbc(i==0, i==1, i==2);
        Bary mbc = Bary(i!=0, i!=1, i!=2) / 2;
        Vector dv = (ptp(pts[(i+2)%3]) - ptp(pts[(i+1)%3])).normalized();
        dv = Vector(dv(1), -dv(0)); // Rotate 90° ccw
        Values vVert, vEdge;
        for(unsigned j = 0; j < 6; ++j)
        {
            vVert(j) = (j == i);
            vEdge(j) = (j == i+3);
        }

        VERIFY(elem.projPoint(i).isApprox(ptp(pts[i]), epsilon));
        VERIFY(elem.bcProj(ptp(pts[i])).isApprox(vbc, epsilon));

        VERIFY(elem.eval(vbc).isApprox(vVert, epsilon));
        VERIFY((elem.jacobian(mbc) * dv).isApprox(vEdge, epsilon));
    }
}


template<typename Point>
void testFvElement(const Point* pts) {
    typedef typename Point::Scalar Scalar;
    typedef FVElement<Scalar> Elem;
    typedef typename Elem::BarycentricCoord Bary;
    typedef typename Elem::Vector Vector;
    typedef typename Elem::Values Values;

    Scalar epsilon = testEpsilon<Scalar>();

    PointToProj<Point> ptp(pts);
    Elem elem(pts);
    for(unsigned i = 0; i < 3; ++i)
    {
        Bary vbc(i==0, i==1, i==2);
        Bary mbc = Bary(i!=0, i!=1, i!=2) / 2;
        Scalar quad = (std::sqrt(Scalar(1) / Scalar(3)) + 1) / 2;
        Bary qbc1 = quad * Bary(i==2, i==0, i==1) + (1 - quad) * Bary(i==1, i==2, i==0);
        Bary qbc2 = (1 - quad) * Bary(i==2, i==0, i==1) + quad * Bary(i==1, i==2, i==0);
        Vector dv = (ptp(pts[(i+2)%3]) - ptp(pts[(i+1)%3])).normalized();
        dv = Vector(dv(1), -dv(0)); // Rotate 90° ccw
        Values vVert, vEdge, vGrad;
        for(unsigned j = 0; j < 9; ++j)
        {
            vVert(j) = (j == i);
            vEdge(j) = (j == i+3);
            vGrad(j) = (j == i+6);
        }

        VERIFY(elem.projPoint(i).isApprox(ptp(pts[i]), epsilon));
        VERIFY(elem.bcProj(ptp(pts[i])).isApprox(vbc, epsilon));

        VERIFY(elem.eval(vbc).isApprox(vVert, epsilon));
        VERIFY(elem.eval(mbc).isApprox(vEdge, epsilon));
        Values gradNode = (elem.jacobian(qbc1) * dv + elem.jacobian(qbc2) * dv) / 2;
        VERIFY(gradNode.isApprox(vGrad, epsilon));
    }
}

template<typename Point>
void testAllElems(unsigned dims = Point::SizeAtCompileTime) {
    cout << "Test linear elements " << dims << "D..." << endl;
    for(int i = 0; i < g_repeat; ++i)
    {
        Point pts[3];
        randomPoints(pts, 3, dims);
        Point p = Point::Random(dims);
        CALL_SUBTEST(( testElement<Point, LinearElement<double> >(pts, p) ));
        CALL_SUBTEST(( testLinearElement<Point>(pts) ));
    }
    cout << "Ok!" << endl;
    cout << "Test quadratic elements " << dims << "D..." << endl;
    for(int i = 0; i < g_repeat; ++i)
    {
        Point pts[3];
        randomPoints(pts, 3, dims);
        Point p = Point::Random(dims);
        CALL_SUBTEST(( testElement<Point, QuadraticElement<double> >(pts, p) ));
        CALL_SUBTEST(( testQuadraticElement<Point>(pts) ));
    }
    cout << "Ok!" << endl;
    cout << "Test morley elements " << dims << "D..." << endl;
    for(int i = 0; i < g_repeat; ++i)
    {
        Point pts[3];
        randomPoints(pts, 3, dims);
        Point p = Point::Random(dims);
        CALL_SUBTEST(( testElement<Point, MorleyElement<double> >(pts, p) ));
        CALL_SUBTEST(( testElementHessian<Point, MorleyElement<double> >(pts, p) ));
        CALL_SUBTEST(( testMorleyElement<Point>(pts) ));
    }
    cout << "Ok!" << endl;
    cout << "Test fv elements... " << dims << "D" << endl;
    for(int i = 0; i < g_repeat; ++i)
    {
        Point pts[3];
        randomPoints(pts, 3, dims);
        Point p = Point::Random(dims);
        CALL_SUBTEST(( testElement<Point, FVElement<double> >(pts, p) ));
        CALL_SUBTEST(( testElementHessian<Point, FVElement<double> >(pts, p) ));
        CALL_SUBTEST(( testFvElement<Point>(pts) ));
    }
    cout << "Ok!" << endl;
}

int main(int argc, char** argv)
{
    if(!init_testing(argc, argv))
    {
        return EXIT_FAILURE;
    }

    CALL_SUBTEST(testAllElems<Eigen::Vector2d >());
    CALL_SUBTEST(testAllElems<Eigen::Vector3d >());
    CALL_SUBTEST(testAllElems<Eigen::VectorXd >(6));
}
