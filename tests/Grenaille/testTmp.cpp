
#include "../common/testing.h"
#include "../common/testUtils.h"

#include <unsupported/Eigen/AutoDiff>

using namespace std;
using namespace Grenaille;



template<typename ScalarT, class VectorT, class MatrixT>
struct DistanceFunction
{
    static ScalarT dist(const VectorT& q, ScalarT t) {return q.norm()/t;}

    static VectorT dx_dist(const VectorT& q, ScalarT t) {return -q/(t*q.norm());}
    static ScalarT dt_dist(const VectorT& q, ScalarT t) {return -q.norm()/(t*t);}

    static MatrixT d2x_dist(const VectorT& q, ScalarT t) {return ScalarT(1.)/(t*q.norm())*( MatrixT::Identity() - q*q.transpose()/(q.norm()*q.norm()) );}
    static ScalarT d2t_dist(const VectorT& q, ScalarT t) {return ScalarT(2.)*q.norm()/(t*t*t);}

    static VectorT d2tx_dist(const VectorT& q, ScalarT t) {return q/(t*t*q.norm());}
};


template<typename DataPoint, typename WeightKernel>
void testFunctionAutoDiff()
{
    // Define related structure
    typedef typename WeightKernel::Scalar ScalarDiff;
    typedef typename ScalarDiff::Scalar Scalar;
    typedef typename DataPoint::VectorType VectorTypeDiff;
    typedef typename DataPoint::MatrixType MatrixTypeDiff;

    Scalar epsilon = testEpsilon<Scalar>();
    Scalar scale   = Eigen::internal::random<Scalar>(0.10, 10.0);
    ScalarDiff t   = ScalarDiff(scale, DataPoint::Dim+1, 0);

    // random center
    VectorTypeDiff center = Eigen::internal::random<Scalar>(1, 10000)*VectorTypeDiff::Random();
    VectorTypeDiff x;
    for(int i=0; i<DataPoint::Dim; ++i) {
        x[i] = ScalarDiff(center[i].value(), DataPoint::Dim+1, i+1);
    }

    // rejection sampling inside a sphere of radius equal to scale
    VectorTypeDiff p = scale*VectorTypeDiff::Random();
    while ( p.norm() < 0.001*scale || 0.999*scale < p.norm())
        p = scale*VectorTypeDiff::Random();
    p += center;

    // centered query
    VectorTypeDiff q = p-x;

    DistanceFunction<ScalarDiff, VectorTypeDiff, MatrixTypeDiff> D;

    // Reference
    ScalarDiff     d         = D.dist(q,t);
    VectorTypeDiff dx_dist   = D.dx_dist(q,t);
    ScalarDiff     dt_dist   = D.dt_dist(q,t);
    MatrixTypeDiff d2x_dist  = D.d2x_dist(q,t);
    ScalarDiff     d2t_dist  = D.d2t_dist(q,t);
    VectorTypeDiff d2tx_dist = D.d2tx_dist(q,t);

    // AutoDiff
    VectorTypeDiff dx_dist_;
    ScalarDiff     dt_dist_;
    MatrixTypeDiff d2x_dist_;
    ScalarDiff     d2t_dist_;
    VectorTypeDiff d2tx_dist_;
    VectorTypeDiff d2xt_dist_;

    dx_dist_ = d.derivatives().template tail<DataPoint::Dim>();
    dt_dist_ = d.derivatives()[0];

    d2t_dist_ = dt_dist.derivatives()[0];
    for(int i=0; i<DataPoint::Dim; ++i)
    {
        d2tx_dist_[i] = dx_dist[i].derivatives()[0];
        d2x_dist_.col(i) = dx_dist[i].derivatives().template tail<DataPoint::Dim>();
    }
    d2xt_dist_ = dt_dist.derivatives().template tail<DataPoint::Dim>();

    VERIFY( (dx_dist-dx_dist_).array().abs().maxCoeff() < epsilon );
    VERIFY( abs(dt_dist-dt_dist_) < epsilon );
    VERIFY( abs(d2t_dist-d2t_dist_) < epsilon );
    VERIFY( (d2x_dist-d2x_dist_).array().abs().maxCoeff() < epsilon );
    VERIFY( (d2tx_dist-d2tx_dist_).array().abs().maxCoeff() < epsilon );
    VERIFY( (d2tx_dist-d2xt_dist_).array().abs().maxCoeff() < epsilon );
}

template<typename Scalar, int Dim>
void callSubTests()
{
//    typedef PointPositionNormal<Scalar, Dim> DataPoint;

//    typedef SmoothWeightKernel<Scalar> SmoothKernel;
//    typedef ConstantWeightKernel<Scalar> ConstantKernel;

    typedef Eigen::AutoDiffScalar<Eigen::Matrix<Scalar,Dim+1,1>> ScalarDiff;
    typedef PointPositionNormal<ScalarDiff, Dim> DataPointDiff;

    typedef SmoothWeightKernel<ScalarDiff> SmoothKernelDiff;
    typedef ConstantWeightKernel<ScalarDiff> ConstantKernelDiff;

    for(int i = 0; i < g_repeat; ++i)
    {
//        CALL_SUBTEST(( testFunction<DataPoint, SmoothKernel>() ));
//        CALL_SUBTEST(( testFunction<DataPoint, ConstantKernel>() ));

//        CALL_SUBTEST(( testFunctionAutoDiff<DataPointDiff, SmoothKernelDiff>() ));
        CALL_SUBTEST(( testFunctionAutoDiff<DataPointDiff, ConstantKernelDiff>() ));

    }
    cout << "ok" << endl;
}


int main(int argc, char** argv)
{
    if(!init_testing(argc, argv))
    {
        return EXIT_FAILURE;
    }

    cout << "Tmp" << endl;

//    callSubTests<long double, 1>();
//    callSubTests<long double, 2>();
    callSubTests<long double, 3>();
//    callSubTests<long double, 4>();
//    callSubTests<long double, 5>();
}
