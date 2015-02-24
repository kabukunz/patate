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


template<typename Vector>
void randomPoints(Vector* pts, unsigned count)
{
    for(unsigned i = 0; i < count; ++i)
    {
        pts[i] = Vector::Random();
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
    jacobian.col(0) = (  elem.eval(elem.barycentricCoordinates(p + dx))
                       - elem.eval(elem.barycentricCoordinates(p - dx))) / epsilon;
    Vector dy(0, epsilon / 2);
    jacobian.col(1) = (  elem.eval(elem.barycentricCoordinates(p + dy))
                       - elem.eval(elem.barycentricCoordinates(p - dy))) / epsilon;
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
            Bary bcp = elem.barycentricCoordinates(p + dv[d2]);
            Bary bcm = elem.barycentricCoordinates(p - dv[d2]);
            hessian(d1, d2) = (elem.gradient(bi, bcp)(d1) - elem.gradient(bi, bcm)(d1)) / epsilon;
        }
    }
    return hessian;
}


template<typename Elem>
void testElement(const typename Elem::Vector* pts, const typename Elem::Vector& p) {
    typedef typename Elem::Scalar Scalar;
    typedef typename Elem::BarycentricCoord Bary;

    Scalar epsilon = testEpsilon<Scalar>();

    Elem elem(pts);
    Bary bc = elem.barycentricCoordinates(p);

    VERIFY(evalAll(elem, bc).isApprox(elem.eval(bc), epsilon));
    VERIFY(numJacobian(elem, p, epsilon).isApprox(elem.jacobian(bc), epsilon));
    VERIFY(gradientAll(elem, bc).isApprox(elem.jacobian(bc), epsilon));
}


template<typename Elem>
void testElementHessian(const typename Elem::Vector* pts, const typename Elem::Vector& p) {
    typedef typename Elem::Scalar Scalar;
    typedef typename Elem::BarycentricCoord Bary;
    typedef typename Elem::Hessian Hessian;

    enum
    {
        Size = Elem::Values::RowsAtCompileTime
    };

    Scalar epsilon = testEpsilon<Scalar>();

    Elem elem(pts);
    Bary bc = elem.barycentricCoordinates(p);

    Hessian hessians[Size];
    elem.hessian(bc, hessians);
    for(unsigned i = 0; i < Size; ++i)
    {
        Hessian h = elem.hessian(i, bc);
        VERIFY(h.isApprox(hessians[i], epsilon));
        VERIFY(numHessian(elem, i, p, epsilon).isApprox(h, epsilon));
    }
}


template<typename Scalar>
void testLinearElement(const Eigen::Matrix<Scalar, 2, 1>* pts) {
    typedef LinearElement<Scalar> Elem;
    typedef typename Elem::BarycentricCoord Bary;
    typedef typename Elem::Vector Vector;

    Scalar epsilon = testEpsilon<Scalar>();

    Elem elem(pts);
    Vector p = Vector::Random();
    Bary bc = elem.barycentricCoordinates(p);

    VERIFY(evalAll(elem, bc).isApprox(bc, epsilon));
    for(unsigned i = 0; i < 3; ++i)
    {
        VERIFY(elem.point(i) == pts[i]);
        Bary bc(i==0, i==1, i==2);
        VERIFY(elem.barycentricCoordinates(pts[i]).isApprox(bc, epsilon));
        VERIFY(elem.eval(bc).isApprox(bc, epsilon));
    }
}


template<typename Scalar>
void testQuadraticElement(const Eigen::Matrix<Scalar, 2, 1>* pts) {
    typedef QuadraticElement<Scalar> Elem;
    typedef typename Elem::BarycentricCoord Bary;
    typedef typename Elem::Values Values;

    Scalar epsilon = testEpsilon<Scalar>();

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

        VERIFY(elem.point(i) == pts[i]);
        VERIFY(elem.barycentricCoordinates(pts[i]).isApprox(vbc, epsilon));

        VERIFY(elem.eval(vbc).isApprox(vVert, epsilon));
        VERIFY(elem.eval(mbc).isApprox(vEdge, epsilon));
    }
}


template<typename Scalar>
void testMorleyElement(const Eigen::Matrix<Scalar, 2, 1>* pts) {
    typedef MorleyElement<Scalar> Elem;
    typedef typename Elem::BarycentricCoord Bary;
    typedef typename Elem::Vector Vector;
    typedef typename Elem::Values Values;

    Scalar epsilon = testEpsilon<Scalar>();

    Elem elem(pts);
    for(unsigned i = 0; i < 3; ++i)
    {
        Bary vbc(i==0, i==1, i==2);
        Bary mbc = Bary(i!=0, i!=1, i!=2) / 2;
        Vector dv = (pts[(i+2)%3] - pts[(i+1)%3]).normalized();
        dv = Vector(dv(1), -dv(0)); // Rotate 90° ccw
        Values vVert, vEdge;
        for(unsigned j = 0; j < 6; ++j)
        {
            vVert(j) = (j == i);
            vEdge(j) = (j == i+3);
        }

        VERIFY(elem.point(i) == pts[i]);
        VERIFY(elem.barycentricCoordinates(pts[i]).isApprox(vbc, epsilon));

        VERIFY(elem.eval(vbc).isApprox(vVert, epsilon));
        VERIFY((elem.jacobian(mbc) * dv).isApprox(vEdge, epsilon));
    }
}


template<typename Scalar>
void testFvElement(const Eigen::Matrix<Scalar, 2, 1>* pts) {
    typedef FVElement<Scalar> Elem;
    typedef typename Elem::BarycentricCoord Bary;
    typedef typename Elem::Vector Vector;
    typedef typename Elem::Values Values;

    Scalar epsilon = testEpsilon<Scalar>();

    Elem elem(pts);
    for(unsigned i = 0; i < 3; ++i)
    {
        Bary vbc(i==0, i==1, i==2);
        Bary mbc = Bary(i!=0, i!=1, i!=2) / 2;
        Scalar quad = (std::sqrt(Scalar(1) / Scalar(3)) + 1) / 2;
        Bary qbc1 = quad * Bary(i==2, i==0, i==1) + (1 - quad) * Bary(i==1, i==2, i==0);
        Bary qbc2 = (1 - quad) * Bary(i==2, i==0, i==1) + quad * Bary(i==1, i==2, i==0);
        Vector dv = (pts[(i+2)%3] - pts[(i+1)%3]).normalized();
        dv = Vector(dv(1), -dv(0)); // Rotate 90° ccw
        Values vVert, vEdge, vGrad;
        for(unsigned j = 0; j < 9; ++j)
        {
            vVert(j) = (j == i);
            vEdge(j) = (j == i+3);
            vGrad(j) = (j == i+6);
        }

        VERIFY(elem.point(i) == pts[i]);
        VERIFY(elem.barycentricCoordinates(pts[i]).isApprox(vbc, epsilon));

        VERIFY(elem.eval(vbc).isApprox(vVert, epsilon));
        VERIFY(elem.eval(mbc).isApprox(vEdge, epsilon));
        Values gradNode = (elem.jacobian(qbc1) * dv + elem.jacobian(qbc2) * dv) / 2;
        VERIFY(gradNode.isApprox(vGrad, epsilon));
    }
}


int main(int argc, char** argv)
{
    if(!init_testing(argc, argv))
    {
        return EXIT_FAILURE;
    }

    typedef double Scalar;
    typedef Eigen::Matrix<Scalar, 2, 1> Vector;

    cout << "Test linear elements..." << endl;
    for(int i = 0; i < g_repeat; ++i)
    {
        Vector pts[3];
        randomPoints(pts, 3);
        Vector p = Vector::Random();
        CALL_SUBTEST(( testElement<LinearElement<double> >(pts, p) ));
        CALL_SUBTEST(( testLinearElement<double>(pts) ));
    }
    cout << "Ok!" << endl;
    cout << "Test quadratic elements..." << endl;
    for(int i = 0; i < g_repeat; ++i)
    {
        Vector pts[3];
        randomPoints(pts, 3);
        Vector p = Vector::Random();
        CALL_SUBTEST(( testElement<QuadraticElement<double> >(pts, p) ));
        CALL_SUBTEST(( testQuadraticElement<double>(pts) ));
    }
    cout << "Ok!" << endl;
    cout << "Test morley elements..." << endl;
    for(int i = 0; i < g_repeat; ++i)
    {
        Vector pts[3];
        randomPoints(pts, 3);
        Vector p = Vector::Random();
        CALL_SUBTEST(( testElement<MorleyElement<double> >(pts, p) ));
        CALL_SUBTEST(( testElementHessian<MorleyElement<double> >(pts, p) ));
        CALL_SUBTEST(( testMorleyElement<double>(pts) ));
    }
    cout << "Ok!" << endl;
    cout << "Test fv elements..." << endl;
    for(int i = 0; i < g_repeat; ++i)
    {
        Vector pts[3];
        randomPoints(pts, 3);
        Vector p = Vector::Random();
        CALL_SUBTEST(( testElement<FVElement<double> >(pts, p) ));
        CALL_SUBTEST(( testElementHessian<FVElement<double> >(pts, p) ));
        CALL_SUBTEST(( testFvElement<double>(pts) ));
    }
    cout << "Ok!" << endl;
}
