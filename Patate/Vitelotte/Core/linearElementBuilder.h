#ifndef _LINEAR_ELEMENT_BUILDER_H_
#define _LINEAR_ELEMENT_BUILDER_H_


#include <Eigen/Core>
#include <Eigen/Sparse>


#include "elementBuilderBase.h"


namespace Vitelotte
{


template < class _Mesh, typename _Scalar = typename _Mesh::Scalar >
class LinearElementBuilder : public ElementBuilderBase
{
public:
    typedef _Scalar Scalar;
    typedef _Mesh Mesh;

    typedef Eigen::Matrix<Scalar, Mesh::Dim, 1> Vector;
    typedef Eigen::Triplet<Scalar> Triplet;

    typedef typename Mesh::Face Face;


public:
    inline LinearElementBuilder();

    unsigned nCoefficients(const Mesh& mesh, Face element) const;

    template < typename InIt >
    void addCoefficients(InIt& it, const Mesh& mesh, Face element);
};


} // namespace Vitelotte

#include "linearElementBuilder.hpp"


#endif
