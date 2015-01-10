#ifndef _SINGULAR_ELEMENT_DECORATOR_H_
#define _SINGULAR_ELEMENT_DECORATOR_H_


#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>


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


protected:
    typedef typename Mesh::Face Face;


public:
    inline explicit SingularElementDecorator(const Base& element=Base())
        : Base(element) {}

    unsigned nCoefficients(const Mesh& mesh, Face element) const;

    template < typename InIt >
    void addCoefficients(InIt& it, const Mesh& mesh, Face element);
};


} // namespace Vitelotte

#include "singularElementDecorator.hpp"


#endif
