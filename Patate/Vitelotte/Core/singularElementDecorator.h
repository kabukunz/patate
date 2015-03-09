/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _VITELOTTE_SINGULAR_ELEMENT_DECORATOR_
#define _VITELOTTE_SINGULAR_ELEMENT_DECORATOR_


#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "solverError.h"


namespace Vitelotte
{


template < class _Element >
class SingularElementDecorator : public _Element
{
public:
    typedef _Element Base;

    typedef typename Base::Scalar Scalar;
    typedef typename Base::Mesh Mesh;

    typedef typename Base::Vector Vector;
    typedef typename Base::Triplet Triplet;

    typedef typename Base::Face Face;


public:
    inline explicit SingularElementDecorator(const Base& element=Base())
        : Base(element) {}

    unsigned nCoefficients(const Mesh& mesh, Face element,
                           SolverError* error=0) const;

    template < typename InIt >
    void addCoefficients(InIt& it, const Mesh& mesh, Face element,
                         SolverError* error=0);
};


} // namespace Vitelotte

#include "singularElementDecorator.hpp"


#endif