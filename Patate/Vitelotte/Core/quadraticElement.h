#ifndef _QUADRATIC_ELEMENT_H_
#define _QUADRATIC_ELEMENT_H_


#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>


namespace Vitelotte
{


template < class _Mesh, typename _Scalar = typename _Mesh::Scalar >
class QuadraticElement
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
    inline QuadraticElement();

    unsigned nCoefficients(const Mesh& mesh, Face element) const;
    void addCoefficients(TripletVectorIterator& it,
                         const Mesh& mesh, Face element) const;

private:
    static bool m_matricesInitialized;
    static ElementStiffnessMatrix m_quadM1;
    static ElementStiffnessMatrix m_quadM2;
    static ElementStiffnessMatrix m_quadM3;
};


#include "quadraticElement.hpp"

} // namespace Vitelotte

#endif
