/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

/*!
    \file test/Grenaille/mls_paraboloid_der.cpp
    \brief Test validity of dNormal
 */

#include "../common/testing.h"
#include "../common/testUtils.h"

#include <vector>

using namespace std;
using namespace Grenaille;

template<typename DataPoint, typename Fit, typename WeightFunc> //, typename Fit, typename WeightFunction>
void testFunction(bool _bAddNoise = false)
{
    // Define related structure
    typedef typename DataPoint::Scalar Scalar;
    typedef typename DataPoint::VectorType VectorType;
    typedef typename DataPoint::QuaternionType QuaternionType;

    //generate sampled paraboloid
    int nbPoints = Eigen::internal::random<int>(100, 1000);

    VectorType coef = 100 * VectorType(Eigen::internal::random<Scalar>(-1,1), Eigen::internal::random<Scalar>(-1,1), 0);
    coef = VectorType(2, 0, 0);

//    Scalar analysisScale = Scalar(.001);
    Scalar analysisScale = Scalar(2.);

    Scalar epsilon = testEpsilon<Scalar>();
    if( _bAddNoise ) // relax a bit the testing threshold
        epsilon = Scalar(0.002*MAX_NOISE);

    vector<DataPoint> vectorPoints(nbPoints);

    QuaternionType q;

    for(unsigned int i = 0; i < vectorPoints.size(); ++i)
    {
        vectorPoints[i] = getPointOnParaboloid<DataPoint>(VectorType::Zero(),
                                                          coef,
                                                          q,
                                                          analysisScale*Scalar(1.2));
    }

    // Test at central point if the principal curvature directions are correct
    DataPoint point;
    point.pos() = VectorType::Zero();
    point.normal() = VectorType::Zero();
    point.normal().z() = Scalar(1.);

    {

        Fit fit;
        fit.setWeightFunc(WeightFunc(analysisScale));
        for(int i=0; i<5; ++i)
        {
            fit.init(point.pos());
            fit.compute(vectorPoints.cbegin(), vectorPoints.cend());
            point.pos() = fit.project(point.pos());
        }

        if(fit.isStable())
        {
            VectorType k1v = fit.k1Direction();
            VectorType k2v = fit.k2Direction();
            VectorType normal = fit.normal();
            
            Scalar k1 = fit.k1();
            Scalar k2 = fit.k2();

            VectorType k1Theoritical = VectorType(1,0,0);
            VectorType k2Theoritical = VectorType(0,1,0);
            VectorType normalTheoritical = VectorType(0,0,1);

            Scalar absDot1 = std::abs(k1Theoritical.dot(k1v));
            Scalar absDot2 = std::abs(k2Theoritical.dot(k2v));
            Scalar absDotN = std::abs(normalTheoritical.dot(normal));


            std::cout << "k1="<< k1 << " ## " << k1v.transpose() << " ## err1=" << std::abs(Scalar(1.)-absDot1) << std::endl;
            std::cout << "k2="<< k2 << " ## " << k2v.transpose() << " ## err2=" << std::abs(Scalar(1.)-absDot2) << std::endl;
            std::cout << "N =" << normal.transpose() << " ## errN=" << std::abs(Scalar(1.)-absDotN) << std::endl << std::endl;


            auto tmp = fit.dNormal();

            std::cout << "dNormal:\n" << tmp << std::endl << std::endl;


            //TODO(thib) to be continued...
        }
    }
}

template<typename Scalar, int Dim>
void callSubTests()
{
    typedef PointPositionNormal<Scalar, Dim> Point;
    typedef DistWeightFunc<Point, SmoothWeightKernel<Scalar> > WeightSmoothFunc;
    typedef DistWeightFunc<Point, ConstantWeightKernel<Scalar> > WeightConstantFunc;

    //TODO(thib) delete this
    typedef Basket<Point,WeightSmoothFunc, OrientedSphereFit, OrientedSphereSpaceDer/*, MlsSphereFitDer*/, CurvatureEstimator> FitTest;

    typedef Basket<Point,WeightSmoothFunc,OrientedSphereFit, OrientedSphereSpaceDer, MlsSphereFitDer, CurvatureEstimator> FitSmoothOrientedSpaceMlsDer;
    typedef Basket<Point,WeightConstantFunc,OrientedSphereFit, OrientedSphereSpaceDer, MlsSphereFitDer, CurvatureEstimator> FitConstantOrientedSpaceMlsDer;
    typedef Basket<Point,WeightSmoothFunc,OrientedSphereFit, OrientedSphereScaleSpaceDer, MlsSphereFitDer, CurvatureEstimator> FitSmoothOrientedScaleSpaceMlsDer;
    typedef Basket<Point,WeightConstantFunc,OrientedSphereFit, OrientedSphereScaleSpaceDer, MlsSphereFitDer, CurvatureEstimator> FitConstantOrientedScaleSpaceMlsDer;

    cout << "Testing with perfect primitive (oriented / unoriented)..." << endl;
    for(int i = 0; i < g_repeat; ++i)
    {
        //TODO(thib) delete this
//        CALL_SUBTEST(( testFunction<Point, FitTest, WeightSmoothFunc>() ));

        CALL_SUBTEST(( testFunction<Point, FitSmoothOrientedSpaceMlsDer, WeightSmoothFunc>() ));
//        CALL_SUBTEST(( testFunction<Point, FitConstantOrientedSpaceMlsDer, WeightConstantFunc>() ));
//        CALL_SUBTEST(( testFunction<Point, FitSmoothOrientedScaleSpaceMlsDer, WeightSmoothFunc>() ));
//        CALL_SUBTEST(( testFunction<Point, FitConstantOrientedScaleSpaceMlsDer, WeightConstantFunc>() ));
    }
    cout << "Ok!" << endl;

    cout << "Testing with noise on position and normals (oriented / unoriented)..." << endl;
    for(int i = 0; i < g_repeat; ++i)
    {
//        CALL_SUBTEST(( testFunction<Point, FitSmoothOrientedSpaceMlsDer, WeightSmoothFunc>(true) ));
//        CALL_SUBTEST(( testFunction<Point, FitConstantOrientedSpaceMlsDer, WeightConstantFunc>(true) ));
//        CALL_SUBTEST(( testFunction<Point, FitSmoothOrientedScaleSpaceMlsDer, WeightSmoothFunc>(true) ));
//        CALL_SUBTEST(( testFunction<Point, FitConstantOrientedScaleSpaceMlsDer, WeightConstantFunc>(true) ));
    }
    cout << "Ok!" << endl;
}

int main(int argc, char** argv)
{
    if(!init_testing(argc, argv))
    {
        return EXIT_FAILURE;
    }

    cout << "Test normal estimation" << endl;

    //callSubTests<double, 2>();
//    callSubTests<float, 3>();
    callSubTests<long double, 3>();
//    callSubTests<double, 3>();
}
