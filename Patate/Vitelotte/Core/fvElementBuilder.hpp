/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


#include <Eigen/Dense>

#include "fvElement.h"

#include "fvElementBuilder.h"


namespace Vitelotte
{


template < class _Mesh, typename _Scalar >
FVElementBuilder<_Mesh, _Scalar>::FVElementBuilder(Scalar sigma)
  : m_sigma(sigma)
{
}

template < class _Mesh, typename _Scalar >
unsigned
FVElementBuilder<_Mesh, _Scalar>::
    nCoefficients(const Mesh& mesh, Face element) const
{
    return 81;
}


template < class _Mesh, typename _Scalar >
template < typename InIt >
void
FVElementBuilder<_Mesh, _Scalar>::
    addCoefficients(InIt& it, const Mesh& mesh, Face element)
{
    if(mesh.valence(element) != 3)
    {
        error(STATUS_ERROR, "Non-triangular face");
        return;
    }

    processFV1Element(it, mesh, element);
}

template < class _Mesh, typename _Scalar >
template < typename InIt >
void
FVElementBuilder<_Mesh, _Scalar>::
    processFV1Element(InIt& it, const Mesh& mesh, Face element)
{
    typedef Eigen::Matrix<Scalar, 9, 9> Matrix9;
    Matrix9 sm;

    int nodes[9];

    typename Mesh::HalfedgeAroundFaceCirculator hit = mesh.halfedges(element);
    typename Mesh::HalfedgeAroundFaceCirculator hend = hit;

    bool orient[3];
    Vector p[3];
    --hit;
    for(int i = 0; i < 3; ++i)
    {
        orient[i] = mesh.halfedgeOrientation(*hit);
        nodes[3+i] = mesh.edgeValueNode(*hit).idx();
        nodes[6+i] = mesh.edgeGradientNode(*hit).idx();
        ++hit;
        nodes[i] = mesh.toVertexValueNode(*hit).idx();
        p[i] = mesh.position(mesh.toVertex(*hit)).template cast<Scalar>();
    }

    for(int i = 0; i < 9; ++i)
    {
        if(nodes[i] < 0)
        {
            error(STATUS_ERROR, "Invalid node");
            return;
        }
    }

    typedef FVElement<Scalar> Elem;
    Elem elem(p);

    if(elem.doubleArea() <= 0)
    {
        error(STATUS_WARNING, "Degenerated or reversed triangle");
    }

    typedef Eigen::Array<Scalar, 3, 1> Array3;
    Array3 dx2[9];
    Array3 dy2[9];
    Array3 dxy[9];
    for(int pi = 0; pi < 3; ++pi)
    {
        Vector3 bc((pi == 0)? 0: .5,
                   (pi == 1)? 0: .5,
                   (pi == 2)? 0: .5);
        typename Elem::Matrix2 hessians[9];
        elem.hessian(bc, hessians);

        for(int bi = 0; bi < 9; ++bi)
        {
            dx2[bi](pi) = hessians[bi](0, 0);
            dy2[bi](pi) = hessians[bi](1, 1);
            dxy[bi](pi) = hessians[bi](0, 1);
        }
    }

    for(size_t i = 0; i < 9; ++i)
    {
        for(size_t j = i; j < 9; ++j)
        {
            EIGEN_ASM_COMMENT("MYBEGIN");

            Array3 quadPointValue =
                    (dx2[i]+dy2[i]) * (dx2[j]+dy2[j])
                  + (1.-m_sigma) * (
                        2. * dxy[i] * dxy[j]
                      - dx2[i] * dy2[j]
                      - dx2[j] * dy2[i]);

            Scalar value = quadPointValue.sum() * (elem.doubleArea() / 6);

            EIGEN_ASM_COMMENT("MYEND");

            if((i < 6 || orient[i%3]) != (j < 6 || orient[j%3]))
            {
                value *= -1;
            }

//                *(it++) = Triplet(nodes[i], nodes[j], value);
//                if(i != j)
//                    *(it++) = Triplet(nodes[j], nodes[i], value);
            sm(i, j) = value;
            sm(j, i) = value;
        }
    }

    for(size_t i = 0; i < 9; ++i)
    {
        for(size_t j = 0; j < 9; ++j)
        {
            *(it++) = Triplet(nodes[i], nodes[j], sm(i, j));
        }
    }
}


}
