/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


/*!
    \file test/Shelves/kdtree.cpp
    \brief Test kdtree utility functions
 */

#include "../common/testing.h"
#include "../common/testUtils.h"

#include "Patate/shelves.h"

#include <vector>
#include <algorithm>
#include <chrono>       // std::chrono::seconds, std::chrono::milliseconds
                        // std::chrono::duration_cast

using namespace std;
using namespace Grenaille;

template<typename Scalar, int nbPoints, int nbSeeds>
void testFunction()
{
    using namespace std::chrono;
    using Time = std::chrono::high_resolution_clock;
    using ms = std::chrono::milliseconds;

    using KdTree = Shelves::KdTree<Scalar>;
    KdTree tree (nbPoints);
    using VectorType = typename KdTree::VectorType;

    auto t00 = Time::now();
    for (int i = 0; i != nbPoints; ++i){
        tree.add(VectorType::Random());
    }
    tree.finalize();
    auto t01 = Time::now();

    auto t10 = Time::now();
    KdTree tree2 (tree._getPoints().begin(), tree._getPoints().end());
    auto t11 = Time::now();

    std::cout << "Construction times (msec): per-element (" << std::chrono::duration_cast<ms>(t01-t00).count()
              << "), by copy of pre-ordered elements (" << std::chrono::duration_cast<ms>(t11-t10).count() << ")."
              << std::endl;

    Scalar sqdist = 0.001;

    // generate the seed, and compare kdtree search with bruteforce
    std::list<VectorType, Eigen::aligned_allocator<VectorType> > resultsKd, resultsKd2;
    for (int i = 0; i != nbSeeds; ++i){
        VectorType seed = VectorType::Random();

        tree.doQueryDist ( seed, sqdist, resultsKd );
        tree2.doQueryDist( seed, sqdist, resultsKd2 );

        for (const auto& q : tree._getPoints()){
            if ( (q - seed).squaredNorm() <= sqdist ) {
                auto it = std::find( resultsKd.begin(), resultsKd.end(), q ) ;
                VERIFY ( it != resultsKd.end() );
                resultsKd.erase( it );

                it = std::find( resultsKd2.begin(), resultsKd2.end(), q ) ;
                VERIFY ( it != resultsKd2.end() );
                resultsKd2.erase( it );
            }
        }
        VERIFY ( resultsKd.empty() );
        VERIFY ( resultsKd2.empty() );
    }



}

template<typename Scalar>
void callSubTests()
{

    for(int i = 0; i < g_repeat; ++i)
    {
        CALL_SUBTEST(( testFunction<Scalar, 100000, 100>() ));
    }
}

int main(int argc, char** argv)
{
    if(!init_testing(argc, argv))
    {
        return EXIT_FAILURE;
    }

    cout << "Test Kdtree functions" << endl;
    callSubTests<float>();
    callSubTests<double>();
    callSubTests<long double>();
    cout << "Ok..." << endl;
}
