/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/. 
*/


/*!
 \file test/Grenaille/gls_paraboloid_der.cpp
 \brief Test validity of GLS derivatives for a paraboloid
 */

#include "../common/testing.h"
#include "../common/testUtils.h"

#include <vector>

using namespace std;
using namespace Grenaille;

template<typename DataPoint, typename Fit, typename WeightFunc>
void testFunction()
{
    // Define related structure
    typedef typename DataPoint::Scalar Scalar;
    typedef typename DataPoint::VectorType VectorType;
    //typedef typename DataPoint::MatrixType MatrixType;
    typedef typename DataPoint::QuaternionType QuaternionType;

    //generate sampled paraboloid
    int nbPoints = Eigen::internal::random<int>(10000, 20000);

    VectorType vCenter = VectorType::Random() * Eigen::internal::random<Scalar>(0, 1);
    VectorType vCoef = 1000 * VectorType(Eigen::internal::random<Scalar>(-1,1), Eigen::internal::random<Scalar>(-1,1), 0);

    Scalar analysisScale = Scalar(.001);// / std::max(std::abs(vCoef.x()), std::abs(vCoef.y()));
    vCenter *= analysisScale;
    
    Scalar rotationAngle = Eigen::internal::random<Scalar>(Scalar(0.), Scalar(2 * M_PI));
    VectorType vRotationAxis = VectorType::Random().normalized();
    QuaternionType qRotation = QuaternionType(Eigen::AngleAxis<Scalar>(rotationAngle, vRotationAxis));
    qRotation = qRotation.normalized();

    Scalar epsilon = testEpsilon<Scalar>();
    Scalar kappaEpsilon = 0.01;

    vector<DataPoint> vectorPoints(nbPoints);
    vector<DataPoint> vectorPointsOrigin(nbPoints);
    for(unsigned int i = 0; i < vectorPoints.size(); ++i)
    {
      vectorPointsOrigin[i] = getPointOnParaboloid<DataPoint>(vCenter, vCoef, qRotation, analysisScale*Scalar(1.2), false);
      vectorPoints[i].pos() = qRotation * vectorPointsOrigin[i].pos() + vCenter;
      vectorPoints[i].normal() = qRotation * vectorPointsOrigin[i].normal();
    }

    Fit fit;
    fit.setWeightFunc(WeightFunc(analysisScale));
    VectorType vFittingPoint = vCenter;
    fit.init(vFittingPoint);
    for(typename vector<DataPoint>::iterator it = vectorPoints.begin();
        it != vectorPoints.end();
        ++it)
    {
        fit.addNeighbor(*it);
    }
    fit.finalize();

    {
      // Check derivatives wrt numerical differentiation
      // Use long double for stable numerical differentiation
      typedef long double RefScalar;
      typedef PointPositionNormal<RefScalar, 3> RefPoint;
      typedef DistWeightFunc<RefPoint, SmoothWeightKernel<RefScalar> > RefWeightFunc;
      typedef Basket<RefPoint, RefWeightFunc, OrientedSphereFit, GLSParam, OrientedSphereScaleSpaceDer, GLSDer, GLSCurvatureHelper> RefFit;
      
      vector<RefPoint> refVectorPoints(nbPoints);
      for(unsigned int i = 0; i < vectorPoints.size(); ++i)
      {
        refVectorPoints[i].pos() = vectorPoints[i].pos().template cast<RefScalar>();
        refVectorPoints[i].normal() = vectorPoints[i].normal().template cast<RefScalar>();
      }
      
      // Centered fit:
      RefFit ref_fit;
      ref_fit.setWeightFunc(RefWeightFunc(analysisScale));
      VectorType vFittingPoint = vCenter;
      ref_fit.init(vFittingPoint.template cast<RefScalar>());
      for(typename vector<RefPoint>::iterator it = refVectorPoints.begin();
          it != refVectorPoints.end();
          ++it)
      {
          ref_fit.addNeighbor(*it);
      }
      ref_fit.finalize();
      
      RefScalar der_epsilon = epsilon*10;
//       if(Eigen::internal::is_same<Scalar,float>::value)
//         der_epsilon = epsilon*100;
      // FIXME check whether numerical accuracy can be improved with float
      typename RefFit::VectorArray dUl, dN;
      // typename RefFit::VectorArray dSumP;
      typename RefFit::ScalarArray dUc, dUq, dTau, dKappa;
      Scalar h = Scalar(0.000001)*analysisScale;
      
      // Differentiation along scale, x, y, z:
      for(int k = 0; k<4; ++k)
      {
        RefFit f;
        f.setWeightFunc(RefWeightFunc(analysisScale));
        VectorType vFittingPoint = vCenter;
        if(k==0)
          f.setWeightFunc(RefWeightFunc(analysisScale+h));
        else
          vFittingPoint(k-1) += h;
        f.init(vFittingPoint.template cast<RefScalar>());
        for(typename vector<RefPoint>::iterator it = refVectorPoints.begin(); it != refVectorPoints.end();++it)
          f.addNeighbor(*it);
        f.finalize();
        
        Scalar uc = f.m_uc;
        typename RefFit::VectorType ul = f.m_ul;
        //typename RefFit::VectorType sumP = f.m_sumP;

        // Take into account centered basis:
        if(k>0) uc        += -f.m_ul(k-1) * h + f.m_uq * h * h;
        if(k>0) ul[k-1]   -= 2. * f.m_uq * h;
        // if(k>0) sumP[k-1] += f.m_sumW * h;

        // dSumP.col(k)  = ( sumP      - ref_fit.m_sumP  ) / h;
        dUc(k)        = ( uc        - ref_fit.m_uc    ) / h;
        dUq(k)        = ( f.m_uq    - ref_fit.m_uq    ) / h;
        dUl.col(k)    = ( ul        - ref_fit.m_ul    ) / h;
        dTau(k)       = ( f.tau()   - ref_fit.tau()   ) / h;
        dN.col(k)     = ( f.eta()   - ref_fit.eta()   ) / h;
        dKappa(k)     = ( f.kappa() - ref_fit.kappa() ) / h;
      }

//       std::cout << "== Numerical differentiation ==\n";
//       std::cout << "dUc: "  << dUc << " == " << fit.m_dUc << " ; " << (dUc.template cast<Scalar>()-fit.m_dUc).norm()/dUc.norm() << "\n";
//       std::cout << "dUq: "  << dUq << " == " << fit.m_dUq << " ; " << (dUq.template cast<Scalar>()-fit.m_dUq).norm()/dUq.norm() << "\n";
//       std::cout << "dTau: " << dTau << " == " << fit.dtau() << " ; " << (dTau.template cast<Scalar>()-fit.dtau()).norm()/dTau.norm() << "\n";
//       std::cout << "dKappa: " << dKappa << " == " << fit.dkappa() << " ; " << (dKappa.template cast<Scalar>()-fit.dkappa()).norm()/dKappa.norm() << "\n";
//       std::cout << "dN:\n"  << dN << "\n == \n" << fit.deta() << " ; " << (dN.template cast<Scalar>()-fit.deta()).norm()/dN.norm() << "\n";
//       std::cout << "dUl:\n" << dUl << "\n == \n" << fit.m_dUl << " ; " << (dUl.template cast<Scalar>()-fit.m_dUl).norm()/dUl.norm() << "\n";
//       // std::cout << "dSumP:\n" << dSumP << "\n == \n" << fit.m_dSumP << " ; " << (dSumP.template cast<Scalar>()-fit.m_dSumP).norm()/dSumP.norm() << "\n";
//       std::cout << "eig(dN): " << Eigen::EigenSolver<MatrixType>(dN.template cast<Scalar>().template rightCols<3>()).eigenvalues().real().transpose() << "\n\n";

//       VERIFY( dUc.template cast<Scalar>().isApprox( fit.m_dUc, der_epsilon ) );
//       VERIFY( dUq.template cast<Scalar>().isApprox( fit.m_dUq, der_epsilon ) );
//       VERIFY( (dUl.template cast<Scalar>()-fit.m_dUl).norm() < fit.m_ul.norm() * der_epsilon );
      VERIFY( dTau.template cast<Scalar>().isApprox( fit.dtau(), der_epsilon ) );
      VERIFY( dN.template cast<Scalar>().isApprox( fit.deta(), der_epsilon ) );
      VERIFY( dKappa.template cast<Scalar>().isApprox( fit.dkappa(), der_epsilon ) );
    }
    return;
 
    VERIFY(fit.isStable());
    
    if(fit.isStable())
    {
      Scalar a = vCoef.x();
      Scalar b = vCoef.y();

      Scalar theoricTau = 0;
      VectorType theoricNormal = qRotation * VectorType(0, 0, 1);
      Scalar theoricKmean        = (a + b) * Scalar(.5);
      Scalar theoricAverageKmean = getKappaMean<DataPoint>(vectorPointsOrigin, vFittingPoint, a, b, analysisScale);
      Scalar theoricK1 = std::abs(a)<std::abs(b) ? b : a;
      Scalar theoricK2 = std::abs(a)<std::abs(b) ? a : b;
      Scalar theoricGaussian = a * b;

      Scalar tau = fit.tau();
      VectorType normal = fit.eta();
      Scalar kmean = fit.kappa();

      Scalar kappa1 = fit.GLSk1();
      Scalar kappa2 = fit.GLSk2();
      Scalar kmeanFromK1K2 = (kappa1 + kappa2) * Scalar(.5);
      Scalar gaussian = fit.GLSGaussianCurvature();

//       std::cout << "k1        : " << kappa1 << "  \tref: " << theoricK1 << std::endl;
//       std::cout << "k2        : " << kappa2 << "  \tref: " << theoricK2 << std::endl;
//       std::cout << "kmean     : " << kmean << ", " << kmeanFromK1K2 << "  \tref:" << theoricKmean << " , " << theoricAverageKmean << std::endl;
//       std::cout << "gaussian  : " << gaussian << "  \tref: " << theoricGaussian << std::endl;
//       std::cout << "normal    : " << normal.transpose() << "  \tref: " << theoricNormal.transpose() << std::endl;
      
      VERIFY( Eigen::internal::isMuchSmallerThan(std::abs(tau - theoricTau), Scalar(1.), epsilon) );
      VERIFY( Eigen::internal::isMuchSmallerThan((theoricNormal - normal).norm(), Scalar(1.), epsilon ) );
      VERIFY( Eigen::internal::isMuchSmallerThan(std::abs(theoricAverageKmean - kmeanFromK1K2), std::abs(theoricK1), kappaEpsilon) );
      VERIFY( Eigen::internal::isMuchSmallerThan(std::abs(theoricAverageKmean - kmean), std::abs(theoricK1), kappaEpsilon) );
      
      if(std::abs(std::abs(theoricK1)-std::abs(theoricK2))>kappaEpsilon*std::abs(theoricK1))
      {
        // absolute curvatures are clearly different
        VERIFY( Eigen::internal::isApprox(kappa1, theoricK1, kappaEpsilon) );
        VERIFY( std::abs(kappa2-theoricK2) < kappaEpsilon*std::abs(kappa1) );
      }
      else
      {
        // absolute curvatures are close to each other and therefore their order should be ignored
        VERIFY( Eigen::internal::isApprox(std::abs(kappa1), std::abs(theoricK1), kappaEpsilon) );
        VERIFY( std::abs(std::abs(kappa2)-std::abs(theoricK2)) < kappaEpsilon*std::abs(kappa1) );
      }

      // The errors on k1 and k2 are expected to be of the same order, we thus compare the accuracy of k_gauss to k1^2
      VERIFY( Eigen::internal::isMuchSmallerThan(std::abs(gaussian-theoricGaussian), theoricK1*theoricK1, kappaEpsilon) );
    }
}

template<typename Scalar, int Dim>
void callSubTests()
{
    typedef PointPositionNormal<Scalar, Dim> Point;
    typedef DistWeightFunc<Point, SmoothWeightKernel<Scalar> > WeightSmoothFunc;
    typedef Basket<Point, WeightSmoothFunc, OrientedSphereFit, GLSParam, OrientedSphereScaleSpaceDer, GLSDer, GLSCurvatureHelper> FitSmoothOriented;

    CALL_SUBTEST(( testFunction<Point, FitSmoothOriented, WeightSmoothFunc>() ));
}

int main(int argc, char** argv)
{
    if(!init_testing(argc, argv))
    {
        return EXIT_FAILURE;
    }

    cout << "Test paraboloid fitting..." << endl;

    for(int i = 0; i < g_repeat; ++i)
    {
      callSubTests<float, 3>();
      callSubTests<double, 3>();
      callSubTests<long double, 3>();
    }
}
