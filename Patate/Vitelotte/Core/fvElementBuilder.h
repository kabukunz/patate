#ifndef _FV_ELEMENT_BUILDER_H_
#define _FV_ELEMENT_BUILDER_H_


#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>


namespace Vitelotte
{


template < class _Mesh, typename _Scalar = typename _Mesh::Scalar >
class FVElementBuilder
{
public:
    typedef _Scalar Scalar;
    typedef _Mesh Mesh;

    typedef Eigen::Matrix<Scalar, Mesh::Dim, 1> Vector;
    typedef Eigen::Matrix<Scalar, 6, 6> ElementStiffnessMatrix;
    typedef Eigen::Triplet<Scalar> Triplet;
    typedef Eigen::SparseMatrix<Scalar> StiffnessMatrx;

    typedef std::vector<Triplet> TripletVector;
    typedef typename TripletVector::iterator TripletVectorIterator;

    typedef typename Mesh::Face Face;

public:
    static void initializeMatrices();

public:
    inline FVElementBuilder(Scalar sigma = Scalar(.5));

    unsigned nCoefficients(const Mesh& mesh, Face element) const;

    template < typename InIt >
    void addCoefficients(InIt& it, const Mesh& mesh, Face element) const;

private:
    template < typename InIt >
    void processFV1Element(InIt& it, const Mesh& mesh, Face element) const;

    template < typename InIt >
    void processFV1ElementFlat(InIt& it, const Mesh& mesh, Face element) const;

private:
    Scalar m_sigma;
};


#include "fvElementBuilder.hpp"

} // namespace Vitelotte

#endif
