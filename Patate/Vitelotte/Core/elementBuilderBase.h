/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _VITELOTTE_ELEMENT_BUILDER_BASE_
#define _VITELOTTE_ELEMENT_BUILDER_BASE_


#include <string>

#include <Eigen/Dense>
#include <Eigen/Sparse>


namespace Vitelotte
{


template < class _Mesh, typename _Scalar >
class ElementBuilderBase
{
public:
    typedef _Scalar Scalar;
    typedef _Mesh Mesh;

    typedef Eigen::Matrix<Scalar, Mesh::DimsAtCompileTime, 1> Vector;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
    typedef Eigen::Triplet<Scalar> Triplet;

    typedef typename Mesh::Face Face;

    enum MatrixType {
        MATRIX_SPD,
        MATRIX_SYMETRIC
    };

public:
    inline ElementBuilderBase() : m_size(0) {}

    inline void begin(const Mesh& mesh) { m_size = mesh.nodesSize(); }
    unsigned end(const Mesh& /*mesh*/) { return matrixSize(); }

    unsigned matrixSize() const { return m_size; }

    inline void setRhs(const Mesh& /*mesh*/, Matrix& rhs,
                       SolverError* /*error*/=0) {
        rhs.setZero();
    }

    MatrixType matrixType(const Mesh& /*mesh*/) const {
        return MATRIX_SPD;
    }

protected:
    unsigned m_size;
};


}


#endif
