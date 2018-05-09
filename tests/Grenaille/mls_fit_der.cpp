/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

/*!
    \file test/Grenaille/mls_fit_der.cpp
    \brief Test mls derivatives
 */

#include "../common/testing.h"
#include "../common/testUtils.h"

using namespace std;
using namespace Grenaille;

#include <unsupported/Eigen/AutoDiff>

/// Getter* are helper classes to get access to protected data of the basket

template < class DataPoint, class _WFunctor, typename T = void >
class GetterOrientedSphereFit : public T
{
private:
    typedef T Base;

public:
    typedef typename Base::Scalar       Scalar;
    typedef typename Base::VectorType   VectorType;

public:
    inline const Scalar& uc() const {return Base::m_uc;}
    inline const Scalar& uq() const {return Base::m_uq;}
    inline const VectorType& ul() const {return Base::m_ul;}
public:
    inline const VectorType& sumN() const {return Base::m_sumN;}
    inline const VectorType& sumP() const {return Base::m_sumP;}
    inline const Scalar& sumDotPN() const {return Base::m_sumDotPN;}
    inline const Scalar& sumDotPP() const {return Base::m_sumDotPP;}
    inline const Scalar& sumW() const {return Base::m_sumW;}
    inline const Scalar& nume() const {return Base::m_nume;}
    inline const Scalar& deno() const {return Base::m_deno;}
};

template < class DataPoint, class _WFunctor, typename T = void >
class GetterOrientedSphereDer : public GetterOrientedSphereFit<DataPoint,_WFunctor,T>
{
private:
    typedef GetterOrientedSphereFit<DataPoint,_WFunctor,T> Base;

public:
    typedef typename Base::ScalarArray  ScalarArray;
    typedef typename Base::VectorArray  VectorArray;

public:
    inline const ScalarArray& dUc() const {return Base::m_dUc;}
    inline const ScalarArray& dUq() const {return Base::m_dUq;}
    inline const VectorArray& dUl() const {return Base::m_dUl;}
public:
    inline const VectorArray& dSumN() const {return Base::m_dSumN;}
    inline const VectorArray& dSumP() const {return Base::m_dSumP;}
    inline const ScalarArray& dSumDotPN() const {return Base::m_dSumDotPN;}
    inline const ScalarArray& dSumDotPP() const {return Base::m_dSumDotPP;}
    inline const ScalarArray& dSumW() const {return Base::m_dSumW;}
    inline const ScalarArray& dNume() const {return Base::m_dNume;}
    inline const ScalarArray& dDeno() const {return Base::m_dDeno;}
};

template < class DataPoint, class _WFunctor, typename T = void >
class GetterMlsSphereFitDer : public GetterOrientedSphereDer<DataPoint,_WFunctor,T>
{
private:
    typedef GetterOrientedSphereDer<DataPoint,_WFunctor,T> Base;

public:
    typedef typename Base::Matrix       Matrix;
    typedef typename Base::MatrixArray  MatrixArray;

public:
    inline const Matrix& d2Uc() const {return Base::m_d2Uc;}
    inline const Matrix& d2Uq() const {return Base::m_d2Uq;}
    inline const MatrixArray& d2Ul() const {return Base::m_d2Ul;}
public:
    inline const Matrix& d2SumDotPN() const {return Base::m_d2SumDotPN;}
    inline const Matrix& d2SumDotPP() const {return Base::m_d2SumDotPP;}
    inline const Matrix& d2SumW() const {return Base::m_d2SumW;}
    inline const MatrixArray& d2SumP() const {return Base::m_d2SumP;}
    inline const MatrixArray& d2SumN() const {return Base::m_d2SumN;}
};

template<typename ScalarDiffType>
inline ScalarDiffType testEpsilonAutoDiff()
{
    return testEpsilon<typename ScalarDiffType::Scalar>();
} 

#define CheckScalar(var,dVar)                                                       \
{                                                                                   \
    ScalarArray der = fit.dVar();                                                   \
    ScalarArray autoDiff = ScalarArray::Zero();                                     \
    autoDiff.template tail<Dim>() = fit.var().derivatives().template tail<Dim>();   \
    autoDiff[0] = fit.var().derivatives()[0];                                       \
                                                                                    \
    ScalarDiff diff = (der-autoDiff).array().abs().maxCoeff();                      \
    if( diff > epsilon )                                                            \
    {                                                                               \
        cout << "Error " << #var << "/" << #dVar << ")" << endl;                    \
        cout << "diff     = " << diff  << endl;                                     \
        cout << "der      = " << der   << endl;                                     \
        cout << "autoDiff = " << autoDiff  << endl;                                 \
    }                                                                               \
                                                                                    \
    VERIFY( (der-autoDiff).array().abs().maxCoeff() < epsilon );                    \
}

#define CheckVector(var,dVar)                                                                   \
{                                                                                               \
    VectorArray der = fit.dVar();                                                               \
    VectorArray autoDiff = VectorArray::Zero();                                                 \
    for(int i=0; i<Dim; ++i)                                                                    \
    {                                                                                           \
        autoDiff.row(i).template tail<Dim>() = fit.var()[i].derivatives().template tail<Dim>(); \
        autoDiff.row(i)[0] = fit.var()[i].derivatives()[0];                                     \
    }                                                                                           \
                                                                                                \
    ScalarDiff diff = (der-autoDiff).array().abs().maxCoeff();                                  \
    if( diff > epsilon )                                                                        \
    {                                                                                           \
        cout << "Error " << #var << "/" << #dVar << ")" << endl;                                \
        cout << "diff     = " << diff   << endl;                                                \
        cout << "der      = \n" << der  << endl;                                                \
        cout << "autoDiff = \n" << autoDiff << endl;                                            \
    }                                                                                           \
    VERIFY( (der-autoDiff).array().abs().maxCoeff() < epsilon );                                \
}

#define CheckScalar2(dVar,d2Var)                                                                \
{                                                                                               \
    Matrix der = fit.d2Var();                                                                   \
    Matrix autoDiff = Matrix::Zero();                                                           \
    autoDiff(0,0) = fit.dVar()[0].derivatives()[0];                                             \
    for(int i=0; i<Dim; ++i)                                                                    \
    {                                                                                           \
        autoDiff.template bottomRightCorner<Dim,Dim>().col(i) =                                 \
            fit.dVar()[i+1].derivatives().template tail<Dim>();                                 \
                                                                                                \
        autoDiff.template topRightCorner<1,Dim>()[i] =                                          \
            fit.dVar()[i+1].derivatives()[0];                                                   \
                                                                                                \
        autoDiff.template bottomLeftCorner<Dim,1>()[i] =                                        \
            fit.dVar()[0].derivatives()[i+1];                                                   \
    }                                                                                           \
                                                                                                \
    ScalarDiff diff = (der-autoDiff).array().abs().maxCoeff();                                  \
    if( diff > epsilon )                                                                        \
    {                                                                                           \
        cout << "Error " << #dVar << "/" << #d2Var << ")" << endl;                              \
        cout << "diff     = " << diff   << endl;                                                \
        cout << "der      = \n" << der  << endl;                                                \
        cout << "autoDiff = \n" << autoDiff << endl;                                            \
    }                                                                                           \
    VERIFY( (der-autoDiff).array().abs().maxCoeff() < epsilon );                                \
}

#define CheckVector2(dVar,d2Var)                                            \
{                                                                           \
    MatrixArray der = fit.d2Var();                                          \
    MatrixArray autoDiff = MatrixArray::Zero();                             \
    for(int i=0; i<Dim; ++i)                                                \
    {                                                                       \
        autoDiff.template block<DerDim,DerDim>(0,i*DerDim)(0,0) =           \
            fit.dVar().col(0)[i].derivatives()[0];                          \
                                                                            \
        for(int j=0; j<Dim; ++j)                                            \
        {                                                                   \
                                                                            \
            autoDiff.template block<DerDim,DerDim>(0,i*DerDim).             \
                template bottomLeftCorner<Dim,1>()[j] =                     \
                    fit.dVar().col(0)[i].derivatives()[j+1];                \
                                                                            \
            autoDiff.template block<DerDim,DerDim>(0,i*DerDim).             \
                template topRightCorner<1,Dim>()[j] =                       \
                    fit.dVar()(i,1+j).derivatives()[0];                     \
                                                                            \
            for(int k=0; k<Dim; ++k)                                        \
            {                                                               \
                autoDiff.template block<DerDim,DerDim>(0,i*DerDim).         \
                    template bottomRightCorner<Dim,Dim>().col(j)[k] =       \
                        fit.dVar()(i,1+j).derivatives()[k+1];               \
            }                                                               \
        }                                                                   \
    }                                                                       \
                                                                            \
    ScalarDiff diff = (der-autoDiff).array().abs().maxCoeff();              \
    if( diff > epsilon )                                                    \
    {                                                                       \
        cout << "Error " << #dVar << "/" << #d2Var << ")" << endl;          \
        cout << "diff     = " << diff   << endl;                            \
        cout << "der      = \n" << der  << endl;                            \
        cout << "autoDiff = \n" << autoDiff << endl;                        \
    }                                                                       \
    VERIFY( (der-autoDiff).array().abs().maxCoeff() < epsilon );            \
}

template<typename DataPoint, typename Fit, typename WeightFunc>
void checkDerivativesAutoDiff(const vector<DataPoint> vectorPoints, typename DataPoint::Scalar analysisScale)
{
    typedef typename DataPoint::Scalar     ScalarDiff;
    typedef typename DataPoint::VectorType VectorTypeDiff;

    typedef typename Fit::ScalarArray ScalarArray;
    typedef typename Fit::VectorArray VectorArray;
    typedef typename Fit::Matrix      Matrix;
    typedef typename Fit::MatrixArray MatrixArray;

    const int Dim = DataPoint::Dim;
    const int DerDim = Fit::derDimension();

    const int nbPoints = vectorPoints.size();
    ScalarDiff epsilon = testEpsilonAutoDiff<ScalarDiff>();

    ScalarDiff tDiff = ScalarDiff(analysisScale.value(), DataPoint::Dim+1, 0);

#pragma omp parallel for
    for(int n=0; n<nbPoints; ++n)
    {
        VectorTypeDiff x = vectorPoints[n].pos();
        VectorTypeDiff xDiff;
        for(int i=0; i<DataPoint::Dim; ++i) {
            xDiff[i] = ScalarDiff(x[i].value(), DataPoint::Dim+1, i+1);
        }

        Fit fit;
        fit.setWeightFunc(WeightFunc(tDiff));
        fit.init(xDiff);
        for(int i=0; i<nbPoints; ++i) {
            if(i!=n) fit.addNeighbor(vectorPoints[i]);
        }
        fit.finalize();

        CheckScalar(sumW,dSumW);
        CheckScalar(sumDotPP,dSumDotPP);
        CheckScalar(sumDotPN,dSumDotPN);
        CheckVector(sumP,dSumP);
        CheckVector(sumN,dSumN);

        CheckScalar(nume,dNume);
        CheckScalar(deno,dDeno);

        CheckScalar(uc,dUc);
        CheckVector(ul,dUl);
        CheckScalar(uq,dUq);

        CheckScalar2(dSumW,d2SumW);
        CheckScalar2(dSumDotPP,d2SumDotPP);
        CheckScalar2(dSumDotPN,d2SumDotPN);
        CheckVector2(dSumP,d2SumP);
        CheckVector2(dSumN,d2SumN);

        CheckScalar2(dUc,d2Uc);
        CheckVector2(dUl,d2Ul);
        CheckScalar2(dUq,d2Uq);
    }
}

//template<typename DataPoint, typename Fit, typename WeightFunc>
//void checkDerivatives(const vector<DataPoint> vectorPoints, typename Fit::Scalar analysisScale)
//{
//    typedef typename DataPoint::Scalar Scalar;
//    typedef typename DataPoint::VectorType VectorType;
//    typedef typename DataPoint::MatrixType MatrixType;

//    const Scalar h  = Scalar(1e-5);
//    const Scalar h2 = Scalar(.5)*h;
//    const int nbPoints = vectorPoints.size();
//    Scalar epsilon = testEpsilon<Scalar>();

//    vector<VectorType> diff_dUc(nbPoints);
//    vector<MatrixType> diff_dUl(nbPoints);
//    vector<VectorType> diff_dUq(nbPoints);

//    vector<MatrixType> diff_d2Uq(nbPoints);

//#pragma omp parallel for
//    for(int n=0; n<nbPoints; ++n)
//    {
//        VectorType x = vectorPoints[n].pos();

//        Fit fit;
//        fit.setWeightFunc(WeightFunc(analysisScale));
//        fit.init(x);
//        fit.compute(vectorPoints.cbegin(), vectorPoints.cend());

//        Scalar     uc = fit.m_uc;
//        VectorType ul = fit.m_ul;
//        Scalar     uq = fit.m_uq;

//        VectorType dx_uc = fit.m_dUc.template tail<DataPoint::Dim>();
//        MatrixType dx_ul = fit.m_dUl.template rightCols<DataPoint::Dim>();
//        VectorType dx_uq = fit.m_dUq.template tail<DataPoint::Dim>();

//        MatrixType d2x_uc = fit.m_d2Uc.template bottomRightCorner<DataPoint::Dim,DataPoint::Dim>();
//        Eigen::Matrix<Scalar, DataPoint::Dim, DataPoint::Dim*DataPoint::Dim> d2x_ul;
//        MatrixType d2x_uq = fit.m_d2Uq.template bottomRightCorner<DataPoint::Dim,DataPoint::Dim>();;
//        for(int k=0; k<DataPoint::Dim; ++k)
//        {
//            d2x_ul.template block<DataPoint::Dim,DataPoint::Dim>(0,k*DataPoint::Dim) =
//                fit.m_d2Ul.template block<Fit::DerDim,Fit::DerDim>(0,k*Fit::DerDim).
//                    template bottomRightCorner<DataPoint::Dim,DataPoint::Dim>();
//        }

//        Scalar     dt_uc = fit.m_dUc[0];
//        VectorType dt_ul = fit.m_dUl.col(0);
//        Scalar     dt_uq = fit.m_dUq[0];

//        VectorType dx_uc_;
//        MatrixType dx_ul_;
//        VectorType dx_uq_;

//        MatrixType d2x_uc_;
//        Eigen::Matrix<Scalar, DataPoint::Dim, DataPoint::Dim*DataPoint::Dim> d2x_ul_;
//        MatrixType d2x_uq_;

//        for(int i=0; i<DataPoint::Dim; ++i)
//        {
//            VectorType ei = VectorType::Zero();
//            ei[i] = Scalar(1.);

//            fit.init(x + h*ei);
//            fit.compute(vectorPoints.cbegin(), vectorPoints.cend());
//            Scalar     ucR = fit.m_uc;
//            VectorType ulR = fit.m_ul;
//            Scalar     uqR = fit.m_uq;

//            fit.init(x - h*ei);
//            fit.compute(vectorPoints.cbegin(), vectorPoints.cend());
//            Scalar     ucL = fit.m_uc;
//            VectorType ulL = fit.m_ul;
//            Scalar     uqL = fit.m_uq;

//            dx_uc_[i]     = (ucR-ucL)/(Scalar(2.)*h);
//            dx_ul_.col(i) = (ulR-ulL)/(Scalar(2.)*h);
//            dx_uq_[i]     = (uqR-uqL)/(Scalar(2.)*h);

//            for(int j=0; j<DataPoint::Dim; ++j)
//            {
//                VectorType ej = VectorType::Zero();
//                ej[j] = Scalar(1.);

//                fit.init(x + h2*ei + h2*ej);
//                fit.compute(vectorPoints.cbegin(), vectorPoints.cend());
//                Scalar     ucTR = fit.m_uc;
//                VectorType ulTR = fit.m_ul;
//                Scalar     uqTR = fit.m_uq;

//                fit.init(x + h2*ei - h2*ej);
//                fit.compute(vectorPoints.cbegin(), vectorPoints.cend());
//                Scalar     ucBR = fit.m_uc;
//                VectorType ulBR = fit.m_ul;
//                Scalar     uqBR = fit.m_uq;

//                fit.init(x - h2*ei + h2*ej);
//                fit.compute(vectorPoints.cbegin(), vectorPoints.cend());
//                Scalar     ucTL = fit.m_uc;
//                VectorType ulTL = fit.m_ul;
//                Scalar     uqTL = fit.m_uq;

//                fit.init(x - h2*ei - h2*ej);
//                fit.compute(vectorPoints.cbegin(), vectorPoints.cend());
//                Scalar     ucBL = fit.m_uc;
//                VectorType ulBL = fit.m_ul;
//                Scalar     uqBL = fit.m_uq;

//                d2x_uq_(i,j) = (uqTR-uqBR-uqTL+uqBL)/(h*h);

//                if(std::abs(d2x_uq_(i,j)-d2x_uq(i,j))/std::abs(d2x_uq(i,j)) > epsilon)
//                {
////                    cout << "+uqTR = " << +uqTR << "\n";
////                    cout << "-uqBR = " << -uqBR << "\n";
////                    cout << "-uqTL = " << -uqTL << "\n";
////                    cout << "+uqBL = " << +uqBL << "\n";
////                    cout << "Sum   = " << uqTR-uqBR-uqTL+uqBL << "\n";
////                    cout << "h*h   = " << h*h << "\n";
////                    cout << "res   = " << d2x_uq_(i,j) << "\n";
////                    cout << "true  = " << d2x_uq(i,j) << "\n";
////                    cout << "diff  = " << std::abs(d2x_uq_(i,j)-d2x_uq(i,j)) << "\n";
////                    cout << "rel   = " << std::abs(d2x_uq_(i,j)-d2x_uq(i,j))/std::abs(d2x_uq(i,j)) << "\n";
////                    cout << "eps   = " << epsilon << "\n\n";
//                }
//                else
//                {
////                    cout << "ok!!!!\n";
//                }
//            }


//        }

//        diff_dUc[n] = (dx_uc-dx_uc_).array().abs();
//        diff_dUl[n] = (dx_ul-dx_ul_).array().abs();
//        diff_dUq[n] = (dx_uq-dx_uq_).array().abs();

//        diff_d2Uq[n] = (d2x_uq-d2x_uq_).array().abs();

//        for(int i=0; i<DataPoint::Dim; ++i)
//        {
////            VERIFY( std::abs(dx_uc[i]-dx_uc_[i]) < epsilon );
////            VERIFY( std::abs(dx_uq[i]-dx_uq_[i]) < epsilon );
//            for(int j=0; j<DataPoint::Dim; ++j)
//            {
////                VERIFY( std::abs(dx_ul(i,j)-dx_ul_(i,j)) < epsilon );

//                if(std::abs(d2x_uq(i,j)-d2x_uq_(i,j)) > epsilon )
//                {
////                    cout << "d2x_uq  = " << d2x_uq  << "\n\n";
////                    cout << "d2x_uq_ = " << d2x_uq_ << "\n\n";
////                    cout << "diff    = " << (d2x_uq-d2x_uq_).array().abs() << "\n\n";
////                    cout << "(epsilon=" << epsilon << ")\n";
//                }

////                VERIFY( std::abs(d2x_uq(i,j)-d2x_uq_(i,j)) < epsilon );
//            }
//        }
//    }

//    VectorType mean_dUc = VectorType::Zero();
//    MatrixType mean_dUl = MatrixType::Zero();
//    VectorType mean_dUq = VectorType::Zero();

//    MatrixType mean_d2Uq = MatrixType::Zero();

//    for(int n=0; n<nbPoints; ++n)
//    {
//        mean_dUc += diff_dUc[n];
//        mean_dUl += diff_dUl[n];
//        mean_dUq += diff_dUq[n];

//        mean_d2Uq += diff_d2Uq[n];
//    }
//    mean_dUc /= nbPoints;
//    mean_dUl /= nbPoints;
//    mean_dUq /= nbPoints;

//    mean_d2Uq /= nbPoints;

//    cout << "mean duc = "   << mean_dUc.transpose()     << "\n\n";
//    cout << "mean dul =\n " << mean_dUl                 << "\n\n";
//    cout << "mean duq = "   << mean_dUq.transpose()     << "\n\n";
//    cout << "mean d2uq = \n"   << mean_d2Uq             << "\n\n";
//}





template<typename DataPoint, typename Fit, typename WeightFunc, class CheckFuncT>
void testFunction(CheckFuncT&& checkFunc)
{
    // Define related structure
    typedef typename DataPoint::Scalar Scalar;
    typedef typename DataPoint::VectorType VectorType;
    typedef typename DataPoint::QuaternionType QuaternionType;

    int nbPointsParaboloid = Eigen::internal::random<int>(500, 1000);
    int nbPointsPlane = Eigen::internal::random<int>(500, 1000);
    int nbPointsSphere = Eigen::internal::random<int>(500, 1000);
    int nbPointsRectangle = Eigen::internal::random<int>(500, 1000);

    VectorType coefParaboloid = 10 * VectorType(Eigen::internal::random<Scalar>(-1,1), Eigen::internal::random<Scalar>(-1,1), 0);
    Scalar analysisScaleParaboloid = Scalar(0.01);
    QuaternionType qParaboloid;

    Scalar widthPlane  = Eigen::internal::random<Scalar>(1., 10.);
    Scalar heightPlane = widthPlane;
    Scalar analysisScalePlane = Scalar(50.) * sqrt( widthPlane * heightPlane / nbPointsPlane);
    Scalar centerScalePlane   = Eigen::internal::random<Scalar>(1,10000);
    VectorType centerPlane    = VectorType::Random() * centerScalePlane;
    VectorType directionPlane = VectorType::Random().normalized();

    Scalar radiusSphere = Eigen::internal::random<Scalar>(1,10);
    VectorType centerSphere = VectorType::Random() * Eigen::internal::random<Scalar>(1, 10000);
    Scalar analysisScaleSphere = Scalar(10.) * sqrt(Scalar(4. * M_PI) * radiusSphere * radiusSphere / nbPointsSphere);

    Scalar widthRectangle  = Eigen::internal::random<Scalar>(1., 10.);
    Scalar heightRectangle = widthRectangle;
    Scalar analysisScaleRectangle = Scalar(20.) * sqrt( widthRectangle * heightRectangle / nbPointsRectangle);
    Scalar centerScaleRectangle   = Eigen::internal::random<Scalar>(1,10000);
    VectorType centerRectangle    = VectorType::Random() * centerScaleRectangle;
    VectorType directionRectangle = VectorType::Random().normalized();
    VectorType xAxisRectangle, yAxisRectangle;
    int i0 = -1, i1 = -1, i2 = -1;
    directionRectangle.minCoeff(&i0);
    i1 = (i0+1)%3;
    i2 = (i0+2)%3;
    xAxisRectangle[i0] = 0;
    xAxisRectangle[i1] = directionRectangle[i2];
    xAxisRectangle[i2] = -directionRectangle[i1];
    yAxisRectangle[i0] = directionRectangle[i1]*directionRectangle[i1] + directionRectangle[i2]*directionRectangle[i2];
    yAxisRectangle[i1] = -directionRectangle[i1]*directionRectangle[i0];
    yAxisRectangle[i2] = -directionRectangle[i2]*directionRectangle[i0];

    vector<DataPoint> vectorPointsParaboloid(nbPointsParaboloid);
    vector<DataPoint> vectorPointsPlane(nbPointsParaboloid);
    vector<DataPoint> vectorPointsSphere(nbPointsParaboloid);
    vector<DataPoint> vectorPointsRectangle(nbPointsRectangle);

    for(unsigned int i = 0; i < vectorPointsParaboloid.size(); ++i)
    {
        vectorPointsParaboloid[i] = getPointOnParaboloid<DataPoint>(VectorType::Zero(),
                                                                    coefParaboloid,
                                                                    qParaboloid,
                                                                    Scalar(1.2)*analysisScaleParaboloid,
                                                                    false);
    }
    for(unsigned int i = 0; i < vectorPointsPlane.size(); ++i)
    {
        vectorPointsPlane[i] = getPointOnPlane<DataPoint>(centerPlane,
                                                          directionPlane,
                                                          widthPlane,
                                                          false,
                                                          false);
    }
    for(unsigned int i = 0; i < vectorPointsParaboloid.size(); ++i)
    {
        vectorPointsSphere[i] = getPointOnSphere<DataPoint>(radiusSphere,
                                                            centerSphere,
                                                            false,
                                                            false);
    }
    for(unsigned int i = 0; i < vectorPointsRectangle.size(); ++i)
    {
        vectorPointsRectangle[i] = getPointOnRectangularPlane<DataPoint>(centerRectangle,
                                                                directionRectangle,
                                                                widthRectangle,
                                                                heightRectangle,
                                                                xAxisRectangle,
                                                                yAxisRectangle,
                                                                false);
        vectorPointsRectangle[i].normal() = directionRectangle;
    }

    checkFunc(vectorPointsParaboloid, Scalar(1.2)*analysisScaleParaboloid);
    checkFunc(vectorPointsPlane, analysisScalePlane);
    checkFunc(vectorPointsSphere, analysisScaleSphere);
    checkFunc(vectorPointsRectangle, analysisScaleRectangle);
}


template<typename Scalar, int Dim>
void callSubTests()
{
    typedef PointPositionNormal<Scalar, Dim> Point;
    typedef DistWeightFunc<Point, SmoothWeightKernel<Scalar> > WeightSmoothFunc;
    typedef DistWeightFunc<Point, ConstantWeightKernel<Scalar> > WeightConstantFunc;
    typedef Basket<Point, WeightSmoothFunc, OrientedSphereFit, OrientedSphereScaleSpaceDer, MlsSphereFitDer, GetterMlsSphereFitDer> FitSmoothScaleSpaceMlsDer;
    typedef Basket<Point, WeightConstantFunc, OrientedSphereFit, OrientedSphereScaleSpaceDer, MlsSphereFitDer, GetterMlsSphereFitDer> FitConstantScaleSpaceMlsDer;

    typedef Eigen::AutoDiffScalar<Eigen::Matrix<Scalar,Dim+1,1>> ScalarDiff;
    typedef PointPositionNormal<ScalarDiff, Dim> PointDiff;
    typedef DistWeightFunc<PointDiff, SmoothWeightKernel<ScalarDiff> > WeightSmoothFuncDiff;
    typedef DistWeightFunc<PointDiff, ConstantWeightKernel<ScalarDiff> > WeightConstantFuncDiff;
    typedef Basket<PointDiff, WeightSmoothFuncDiff, OrientedSphereFit, OrientedSphereScaleSpaceDer, MlsSphereFitDer, GetterMlsSphereFitDer> FitSmoothScaleSpaceMlsDerDiff;
    typedef Basket<PointDiff, WeightConstantFuncDiff, OrientedSphereFit, OrientedSphereScaleSpaceDer, MlsSphereFitDer, GetterMlsSphereFitDer> FitConstantScaleSpaceMlsDerDiff;

    for(int i = 0; i < g_repeat; ++i)
    {
//        CALL_SUBTEST(( testFunction<Point, FitSmoothScaleSpaceMlsDer, WeightSmoothFunc>(checkDerivatives<Point, FitSmoothScaleSpaceMlsDer, WeightSmoothFunc>) ));
//        CALL_SUBTEST(( testFunction<Point, FitConstantScaleSpaceMlsDer, WeightConstantFunc>(checkDerivatives<Point, FitSmoothScaleSpaceMlsDer, WeightSmoothFunc>) ));

        CALL_SUBTEST(( testFunction<PointDiff, FitSmoothScaleSpaceMlsDer, WeightSmoothFunc>(checkDerivativesAutoDiff<PointDiff, FitSmoothScaleSpaceMlsDerDiff, WeightSmoothFuncDiff>) ));
        CALL_SUBTEST(( testFunction<PointDiff, FitConstantScaleSpaceMlsDer, WeightConstantFunc>(checkDerivativesAutoDiff<PointDiff, FitSmoothScaleSpaceMlsDerDiff, WeightSmoothFuncDiff>) ));
    }
    cout << "ok" << endl;
}



int main(int argc, char** argv)
{
    if(!init_testing(argc, argv))
    {
        return EXIT_FAILURE;
    }

    cout << "Verify mls derivatives" << endl;

//    callSubTests<long double, 1>();
//    callSubTests<long double, 2>();
    callSubTests</*long */double, 3>();
//    callSubTests<long double, 4>();
//    callSubTests<long double, 5>();
}
