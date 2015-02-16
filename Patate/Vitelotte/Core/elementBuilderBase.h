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

    typedef Eigen::Matrix<Scalar, Mesh::Dim, 1> Vector;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
    typedef Eigen::MatrixXi IndexMap;
    typedef Eigen::Triplet<Scalar> Triplet;

    typedef typename Mesh::Face Face;

    enum MatrixType {
        MATRIX_SPD,
        MATRIX_SYMETRIC
    };

    enum Status
    {
        STATUS_OK,
        STATUS_WARNING,
        STATUS_ERROR
    };

public:
    inline ElementBuilderBase() : m_status(STATUS_OK), m_errorString() {}
    inline Status status() const { return m_status; }
    inline const std::string& errorString() const { return m_errorString; }
    inline void resetStatus() { m_status = STATUS_OK; m_errorString.clear(); }

    inline void setRhs(const Mesh& /*mesh*/, IndexMap /*imap*/, Matrix& rhs) {
        rhs.setZero();
    }

    MatrixType matrixType(const Mesh& /*mesh*/) const {
        return MATRIX_SPD;
    }

protected:
    inline void error(Status status, const std::string& errorString)
    {
        if(m_status > status)
            return;
        m_status = status;
        m_errorString = errorString;
    }

private:
    Status m_status;
    std::string m_errorString;
};


}


#endif
