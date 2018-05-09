/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

/*!
    \file test/Grenaille/dnormal_paraboloid.cpp
    \brief Test validity of dnormal
 */

#include "../common/testing.h"
#include "../common/testUtils.h"

#include <vector>

using namespace std;
using namespace Grenaille;



#include <fstream>
template<typename VectorArrayT>
void exportOBJ(const std::string& filename, const VectorArrayT& data)
{
    std::ofstream file(filename);
    if(!file.is_open())
    {
        std::cout << "Failed to open file [" << filename << "]" << std::endl;
        return;
    }

    for(const auto& pt : data)
    {
        file << "v " << pt.pos().transpose() << "\n";
    }

    file.close();

    std::cout << "### Data exported to [" << filename << "]" << std::endl;
}

template<typename DataPoint, typename Fit, typename WeightFunc> //, typename Fit, typename WeightFunction>
void testFunction(bool _bAddNoise = false)
{
    // Define related structure
    typedef typename DataPoint::Scalar Scalar;
    typedef typename DataPoint::VectorType VectorType;
    typedef typename DataPoint::MatrixType MatrixType;
    typedef typename DataPoint::QuaternionType QuaternionType;

    //generate sampled paraboloid
    int nbPoints = Eigen::internal::random<int>(1000, 10000);

//    VectorType coef = 100 * VectorType(Eigen::internal::random<Scalar>(-1,1), Eigen::internal::random<Scalar>(-1,1), 0);

    // create saddle surface
    VectorType coef = VectorType::Zero();
    coef[0] = Eigen::internal::random<Scalar>(1,100);
    coef[1] = -coef[0];

    Scalar analysisScale = Eigen::internal::random<Scalar>(0.5, 1.0)/coef[0];

    Scalar epsilon = 0.1; //testEpsilon<Scalar>();
    if( _bAddNoise ) // relax a bit the testing threshold
       epsilon = Scalar(0.002*MAX_NOISE);

    vector<DataPoint> vectorPoints(nbPoints);

    QuaternionType q;

    for(unsigned int i = 0; i < vectorPoints.size(); ++i)
    {
       vectorPoints[i] = getPointOnParaboloid<DataPoint>(VectorType::Zero(),
                                                         coef,
                                                         q,
                                                         analysisScale*Scalar(1.2),
                                                         _bAddNoise);
    }

    // Test at central point if dNormal is correct
    DataPoint point;
    point.pos() = VectorType::Zero();
    point.normal() = VectorType::Zero();
    point.normal().z() = Scalar(0.1);
    {
       Fit fit;
       fit.setWeightFunc(WeightFunc(analysisScale));
       for(int i=0; i<   10   ; ++i)
       {
            fit.init(point.pos());
            fit.compute(vectorPoints.cbegin(), vectorPoints.cend());
            if(fit.isStable())
                point.pos() = fit.project(point.pos());
       }

       if(fit.isStable())
       {
            MatrixType dN = fit.dNormal().template rightCols<DataPoint::Dim>();
            Eigen::SelfAdjointEigenSolver<MatrixType> solver(dN);

            VectorType eigValues = solver.eigenvalues();
            MatrixType eigVectors = solver.eigenvectors();


            if( ! (std::abs(eigValues[1]) < epsilon                             &&
                   eigValues[0] < Scalar(0.)                                    &&
                   eigValues[2] > Scalar(0.)                                    &&
                   std::abs(eigValues[0]+eigValues[2])/std::abs(2*eigValues[0]) < epsilon ))
            {
                cout << eigValues << endl;
                exportOBJ("what.obj", vectorPoints);
            }


            VERIFY( std::abs(eigValues[1]) < epsilon );
            VERIFY( eigValues[0] < Scalar(0.) );
            VERIFY( eigValues[2] > Scalar(0.) );
            VERIFY( std::abs(eigValues[0]+eigValues[2])/std::abs(2*eigValues[0]) < epsilon );

            VERIFY( std::abs(Scalar(1.)-std::abs(VectorType(0,1,0).dot(eigVectors.col(0)))) < epsilon );
            VERIFY( std::abs(Scalar(1.)-std::abs(VectorType(0,0,1).dot(eigVectors.col(1)))) < epsilon );
            VERIFY( std::abs(Scalar(1.)-std::abs(VectorType(1,0,0).dot(eigVectors.col(2)))) < epsilon );
       }
       else
       {
           cout << "Warning: unstable fit" << endl;
       }
    }
}

template<typename Scalar, int Dim>
void callSubTests()
{
    typedef PointPositionNormal<Scalar, Dim> Point;
    typedef DistWeightFunc<Point, SmoothWeightKernel<Scalar> > WeightSmoothFunc;
    typedef DistWeightFunc<Point, ConstantWeightKernel<Scalar> > WeightConstantFunc;

    typedef Basket<Point,WeightSmoothFunc,OrientedSphereFit, OrientedSphereSpaceDer> FitSmoothOrientedSpaceDer;
    typedef Basket<Point,WeightConstantFunc,OrientedSphereFit, OrientedSphereSpaceDer> FitConstantOrientedSpaceDer;
    typedef Basket<Point,WeightSmoothFunc,OrientedSphereFit, OrientedSphereScaleSpaceDer> FitSmoothOrientedScaleSpaceDer;
    typedef Basket<Point,WeightConstantFunc,OrientedSphereFit, OrientedSphereScaleSpaceDer> FitConstantOrientedScaleSpaceDer;
    typedef Basket<Point,WeightSmoothFunc,OrientedSphereFit, OrientedSphereSpaceDer, MlsSphereFitDer> FitSmoothOrientedSpaceMlsDer;
    typedef Basket<Point,WeightConstantFunc,OrientedSphereFit, OrientedSphereSpaceDer, MlsSphereFitDer> FitConstantOrientedSpaceMlsDer;
    typedef Basket<Point,WeightSmoothFunc,OrientedSphereFit, OrientedSphereScaleSpaceDer, MlsSphereFitDer> FitSmoothOrientedScaleSpaceMlsDer;
    typedef Basket<Point,WeightConstantFunc,OrientedSphereFit, OrientedSphereScaleSpaceDer, MlsSphereFitDer> FitConstantOrientedScaleSpaceMlsDer;

    cout << "Testing with perfect plane (oriented / unoriented)..." << endl;
    for(int i = 0; i < g_repeat; ++i)
    {
        CALL_SUBTEST(( testFunction<Point, FitSmoothOrientedSpaceDer, WeightSmoothFunc>() ));
        CALL_SUBTEST(( testFunction<Point, FitConstantOrientedSpaceDer, WeightConstantFunc>() ));
        CALL_SUBTEST(( testFunction<Point, FitSmoothOrientedScaleSpaceDer, WeightSmoothFunc>() ));
        CALL_SUBTEST(( testFunction<Point, FitConstantOrientedScaleSpaceDer, WeightConstantFunc>() ));
//        CALL_SUBTEST(( testFunction<Point, FitSmoothOrientedSpaceMlsDer, WeightSmoothFunc>() ));
//        CALL_SUBTEST(( testFunction<Point, FitConstantOrientedSpaceMlsDer, WeightConstantFunc>() ));
//        CALL_SUBTEST(( testFunction<Point, FitSmoothOrientedScaleSpaceMlsDer, WeightSmoothFunc>() ));
//        CALL_SUBTEST(( testFunction<Point, FitConstantOrientedScaleSpaceMlsDer, WeightConstantFunc>() ));
    }
    cout << "Ok!" << endl;
}

int main(int argc, char** argv)
{
    if(!init_testing(argc, argv))
    {
        return EXIT_FAILURE;
    }

    cout << "Test dnormal" << endl;

//    callSubTests<float, 3>();
    callSubTests<long double, 3>();
//    callSubTests<double, 3>();
}
