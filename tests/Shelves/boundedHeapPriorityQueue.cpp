/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


/*!
    \file tests/Shelves/boundedHeapPriorityQueue.cpp
    \brief Test Priority Queue utility functions
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

template<typename Scalar, int size>
void testFunction()
{

//! [Using BoundedHeapMinPriorityQueue]
    using namespace Shelves;


    BoundedHeapMinPriorityQueue <int, float> q;

    q.reserve( size );
    int invId = 0xffffffff;
    q.insert(invId, std::numeric_limits<float>::max());

    for (unsigned int j = 0; j != 2*size; j++){
        float tmp = rand() % 10 + 1;
        cout << "insert " << tmp << " in position " << j << std::endl;
        q.insert( j, tmp );
    }

    for(auto e : q)
        cout << e.index << " : " << e.weight << std::endl;;

    // Check the max element is not in the queue anymore
    auto it = std::find_if( q.begin(), q.end(), [invId](auto el) { return el.index == invId;} );
    VERIFY(it == q.end());

//! [Using BoundedHeapMinPriorityQueue]

}

template<typename Scalar>
void callSubTests()
{

    for(int i = 0; i < 1; ++i)
    {
        CALL_SUBTEST(( testFunction<Scalar, 10>() ));
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
    //callSubTests<double>();
    //callSubTests<long double>();
    cout << "Ok..." << endl;
}
