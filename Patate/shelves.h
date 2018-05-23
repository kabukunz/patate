/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


#ifndef _PATATE_SHELVES_
#define _PATATE_SHELVES_

// First inclue Eigen Core
#include <Eigen/Core>

// Include common stuff
#include "common/defines.h"

// not supported on cuda
#ifndef __CUDACC__
#include "Shelves/src/kdtree.h"
#include "Shelves/src/boundedHeapPriorityQueue.h"
#endif


// Include Shelves Algorithms

#endif //_PATATE_SHELVES_
