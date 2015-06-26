/*
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


int main(int argc, char** argv)
{
    if(!init_testing(argc, argv))
    {
        return EXIT_FAILURE;
    }

    typedef float Scalar;
    typedef Eigen::Matrix<Scalar, 2, 1> Vector;
    typedef BezierSegment<Vector> BS;

    Scalar epsilon = Eigen::NumTraits<Scalar>::dummy_precision();

    BS seg;
    VERIFY(seg.type() == BEZIER_EMPTY);

    Vector points[4];
    for(int i=0; i<4; ++i) points[i] = Vector::Random();

    seg = BS(BEZIER_LINEAR, points);
    VERIFY(seg.type() == BEZIER_LINEAR);
    VERIFY(seg.point(0) == points[0]);
    VERIFY(seg.point(1) == points[1]);

    BS first;
    BS second;
    seg.split(.2, first, second);
    VERIFY(first.type() == BEZIER_LINEAR);
    VERIFY(second.type() == BEZIER_LINEAR);
    VERIFY(first.point(0) == seg.point(0));
    VERIFY(second.point(1) == seg.point(1));
    VERIFY(first.point(1) == second.point(0));
    Vector mid = .8f * seg.point(0) + .2f *  seg.point(1);
    VERIFY( ((first.point(1) - mid).array() < epsilon).all() );

    seg = BS(BEZIER_QUADRATIC, points);
    seg.split(.5, first, second);
    VERIFY(first.type() == BEZIER_QUADRATIC);
    VERIFY(second.type() == BEZIER_QUADRATIC);
    VERIFY(first.point(0) == seg.point(0));
    VERIFY(second.point(2) == seg.point(2));
    VERIFY(first.point(2) == second.point(0));
}
