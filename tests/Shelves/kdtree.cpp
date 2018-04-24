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

using namespace std;
using namespace Grenaille;

template<typename Scalar, int nbPoints, int nbSeeds>
void testFunction()
{
    using KdTree = Shelves::KdTree<Scalar>;
    KdTree tree (nbPoints);
    using VectorType = typename KdTree::VectorType;

    for (int i = 0; i != nbPoints; ++i){
        tree.add(VectorType::Random());
    }
    tree.finalize();

    Scalar sqdist = 0.001;

    // generate the seed, and compare kdtree search with bruteforce
    std::list<VectorType, Eigen::aligned_allocator<VectorType> > resultsKd;
    for (int i = 0; i != nbSeeds; ++i){
        VectorType seed = VectorType::Random();

        tree.doQueryDist( seed, sqdist, resultsKd );

        for (const auto& q : tree._getPoints()){
            if ( (q - seed).squaredNorm() <= sqdist ) {
                auto it = std::find( resultsKd.begin(), resultsKd.end(), q ) ;
                VERIFY ( it != resultsKd.end() );
                resultsKd.erase( it );
            }
        }
        VERIFY ( resultsKd.empty() );
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
