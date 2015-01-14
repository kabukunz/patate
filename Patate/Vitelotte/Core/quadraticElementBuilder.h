#ifndef _VITELOTTE_QUADRATIC_ELEMENT_BUILDER_
#define _VITELOTTE_QUADRATIC_ELEMENT_BUILDER_


#include <Eigen/Core>
#include <Eigen/Sparse>


#include "elementBuilderBase.h"


namespace Vitelotte
{


template < class _Mesh, typename _Scalar = typename _Mesh::Scalar >
class QuadraticElementBuilder : public ElementBuilderBase
{
public:
    typedef _Scalar Scalar;
    typedef _Mesh Mesh;

    typedef Eigen::Matrix<Scalar, Mesh::Dim, 1> Vector;
    typedef Eigen::Triplet<Scalar> Triplet;

    typedef typename Mesh::Face Face;


protected:
    typedef Eigen::Matrix<Scalar, 6, 6> ElementStiffnessMatrix;

protected:
    static void initializeMatrices();

public:
    inline QuadraticElementBuilder();

    unsigned nCoefficients(const Mesh& mesh, Face element) const;

    template < typename InIt >
    void addCoefficients(InIt& it, const Mesh& mesh, Face element);

private:
    static bool m_matricesInitialized;
    static ElementStiffnessMatrix m_quadM1;
    static ElementStiffnessMatrix m_quadM2;
    static ElementStiffnessMatrix m_quadM3;
};


} // namespace Vitelotte

#include "quadraticElementBuilder.hpp"


#endif
