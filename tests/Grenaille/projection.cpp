/*
 Copyright (C) 2014 Nicolas Mellado <nmellado0@gmail.com>

 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


/*!
 \file test/Grenaille/projection.cpp
 \brief Test validity of the direct projection on an algebraic sphere
 */

#include "../common/testing.h"
#include "../common/testUtils.h"

using namespace std;
using namespace Grenaille;

template < class DataPoint, class _WFunctor, typename T = void >
class DirectProjection : public T
{
    private:
    typedef T Base;

public:
    /*! \brief Scalar type inherited from DataPoint*/
    typedef typename Base::Scalar     Scalar;
    /*! \brief Vector type inherited from DataPoint*/
    typedef typename Base::VectorType VectorType;

public:
    //! \brief Project a point on the sphere using a closed form expression
    MULTIARCH inline VectorType project(const VectorType& _q) const
    {
        MULTIARCH_STD_MATH(sqrt);

        // turn to centered basis
        const VectorType lq = _q-Base::basisCenter();

        Scalar potential = Base::m_uc + lq.dot(Base::m_ul) + Base::m_uq * lq.squaredNorm();
        VectorType grad = Base::m_ul + Scalar(2) * Base::m_uq * lq;
        Scalar norm = grad.norm();

        Scalar t;
        if(Base::isPlane())
        {
            t = - potential / (norm*norm);
        }
        else
        {
            t = - (norm - sqrt(norm*norm - Scalar(4) * Base::m_uq * potential)) / (Scalar(2) * Base::m_uq * norm);
        }
        return Base::basisCenter() + lq + t * grad;
    }
};

template<typename T>
void print();

template<> void print<float>(){std::cout << "float";}
template<> void print<double>(){std::cout << "double";}
template<> void print<long double>(){std::cout << "long double";}

template<typename DataPoint, typename WeightFunc> 
void testFunction()
{
    // Define related structure
    typedef typename DataPoint::Scalar Scalar;
    typedef typename DataPoint::VectorType VectorType;
    typedef typename DataPoint::QuaternionType QuaternionType;
    typedef Basket<DataPoint, WeightFunc, OrientedSphereFit, DirectProjection> Fit;

    //generate samples
    int nbPoints = Eigen::internal::random<int>(100, 1000);
    int nbPointsFit = 50;

    // equal probability of having a plane or a random quadric 
    VectorType coeff = 5 * VectorType::Random();
    if(Eigen::internal::random<Scalar>(0., 1.) < Scalar(0.5))
    {
        coeff = VectorType::Zero();
    }
    Scalar width = Eigen::internal::random<Scalar>(1., 10.);
    VectorType center = 1000 * VectorType::Random();

    Scalar zmax = std::abs((coeff[0] + coeff[1]) * width*width);
    Scalar analysisScale = std::sqrt(zmax*zmax + width*width);

    Scalar epsilon = testEpsilon<Scalar>();

    Fit fit;
    fit.setWeightFunc(WeightFunc(analysisScale));
    fit.init(center);

    for(int i = 0; i < nbPointsFit; ++i)
    {
        DataPoint p = getPointOnParaboloid<DataPoint>(VectorType(),     // center (not used)
                                                      coeff,
                                                      QuaternionType(), // (not used)
                                                      width,
                                                      false);           // noise
        p.pos() += center;

        fit.addNeighbor(p);
    }

    if(fit.finalize() == STABLE)
    {
        for (int i = 0; i < nbPoints; ++i)
        {
            VectorType p = center + analysisScale * VectorType::Random();
            VectorType proj = fit.project(p);

            // check that the projected point is on the surface 
            VERIFY( fit.potential(proj) < epsilon );
        }
    }
}

template<typename Scalar, int Dim>
void callSubTests()
{
    typedef PointPositionNormal<Scalar, Dim> Point;

    typedef DistWeightFunc<Point, SmoothWeightKernel<Scalar> > WeightSmoothFunc;
    typedef DistWeightFunc<Point, ConstantWeightKernel<Scalar> > WeightConstantFunc;

    cout << "Testing with parabola..." << endl;
    for(int i = 0; i < g_repeat; ++i)
    {
        CALL_SUBTEST(( testFunction<Point, WeightSmoothFunc>() ));
        CALL_SUBTEST(( testFunction<Point, WeightConstantFunc>() ));
    }
    cout << "Ok!" << endl;
}

int main(int argc, char** argv)
{
    if(!init_testing(argc, argv))
    {
        return EXIT_FAILURE;
    }

    cout << "Test projection for different baskets..." << endl;

    // callSubTests<float, 3>();
    callSubTests<double, 3>();
    callSubTests<long double, 3>();
}
