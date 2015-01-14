/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <Eigen/Dense>

#include "morleyElementBuilder.h"


namespace Vitelotte
{


template < class _Mesh, typename _Scalar >
MorleyElementBuilder<_Mesh, _Scalar>::MorleyElementBuilder(Scalar sigma)
    : m_sigma(sigma)
{
}

template < class _Mesh, typename _Scalar >
unsigned
MorleyElementBuilder<_Mesh, _Scalar>::
    nCoefficients(const Mesh& /*mesh*/, Face /*element*/) const
{
    return 36;
}


template < class _Mesh, typename _Scalar >
template < typename InIt >
void
MorleyElementBuilder<_Mesh, _Scalar>::
    addCoefficients(InIt& it, const Mesh& mesh, Face element)
{
    assert(mesh.valence(element) == 3);

    Vector v[3];
    bool orient[3];
    int nodes[6];

    typename Mesh::HalfedgeAroundFaceCirculator hit = mesh.halfedges(element);
    --hit;
    for(int i = 0; i < 3; ++i)
    {
        v[i] = (mesh.position(mesh.toVertex(*hit)) -
                mesh.position(mesh.fromVertex(*hit))).template cast<Scalar>();
        orient[i] = mesh.halfedgeOrientation(*hit);
        nodes[i+3] = mesh.edgeGradientNode(*hit).idx();
        ++hit;
        nodes[i] = mesh.vertexValueNode(*hit).idx();
    }

    for(int i = 0; i < 6; ++i)
    {
        if(nodes[i] < 0)
        {
            error(StatusError, "Invalid node");
            return;
        }
    }

    typedef Eigen::Matrix<Scalar, 6, 1> Vector6;

    Vector6 dx2;
    Vector6 dxy;
    Vector6 dy2;

    Scalar l0 = v[0].norm();
    Scalar l1 = v[1].norm();
    Scalar l2 = v[2].norm();

    Eigen::Matrix<Scalar, 2, 2> frame;
    frame.col(0) = v[2] / l2;
    frame(0, 1) = -frame(1, 0);
    frame(1, 1) = frame(0, 0);

    Vector p2 = -frame.partialPivLu().solve(v[1]);

    Scalar p1x = l2;
    Scalar p2x = p2[0];
    Scalar p2y = p2[1];

    Scalar _2delta = p1x * p2y;
    Scalar area = _2delta / 2;

    if(area <= 0)
    {
        error(StatusWarning, "Degenerated or reversed triangle");
    }

    Scalar a0 = p1x*p2y / _2delta;
    Scalar b1 = p2y / _2delta;
    Scalar b0 = -b1;
    Scalar c0 = (p2x - p1x) / _2delta;
    Scalar c1 = -p2x / _2delta;
    Scalar c2 = p1x / _2delta;

    Scalar b0_2 = b0 * b0;
    Scalar b1_2 = b1 * b1;
    Scalar c0_2 = c0 * c0;
    Scalar c1_2 = c1 * c1;
    Scalar c2_2 = c2 * c2;

    Scalar dl0dn1_2delta = v[0].dot(v[1]) / (l1);
    Scalar dl0dn2_2delta = v[0].dot(v[2]) / (l2);

    Scalar dl1dn0_2delta = v[1].dot(v[0]) / (l0);
    Scalar dl1dn2_2delta = v[1].dot(v[2]) / (l2);

    Scalar dl2dn0_2delta = v[2].dot(v[0]) / (l0);
    Scalar dl2dn1_2delta = v[2].dot(v[1]) / (l1);

    dx2[0] = 2 * ( b1_2*dl0dn1_2delta/l1                          +  b0_2);
    dxy[0] = 2 * (b1*c1*dl0dn1_2delta/l1                          + b0*c0);
    dy2[0] = 2 * ( c1_2*dl0dn1_2delta/l1 +  c2_2*dl0dn2_2delta/l2 +  c0_2);
    dx2[1] = 2 * ( b0_2*dl1dn0_2delta/l0                          +  b1_2);
    dxy[1] = 2 * (b0*c0*dl1dn0_2delta/l0                          + b1*c1);
    dy2[1] = 2 * ( c0_2*dl1dn0_2delta/l0 +  c2_2*dl1dn2_2delta/l2 +  c1_2);
    dx2[2] = 2 * ( b0_2*dl2dn0_2delta/l0 +  b1_2*dl2dn1_2delta/l1);
    dxy[2] = 2 * (b0*c0*dl2dn0_2delta/l0 + b1*c1*dl2dn1_2delta/l1);
    dy2[2] = 2 * ( c0_2*dl2dn0_2delta/l0 +  c1_2*dl2dn1_2delta/l1 + c2_2);
    dx2[3] = 2 * _2delta *  b0_2 / l0;
    dxy[3] = 2 * _2delta * b0*c0 / l0;
    dy2[3] = 2 * _2delta *  c0_2 / l0;
    dx2[4] = 2 * _2delta *  b1_2 / l1;
    dxy[4] = 2 * _2delta * b1*c1 / l1;
    dy2[4] = 2 * _2delta *  c1_2 / l1;
    dx2[5] = 0;
    dxy[5] = 0;
    dy2[5] = 2 * _2delta *  c2_2 / l2;

    for(int i = 0; i < 6; ++i)
    {
        for(int j = 0; j < 6; ++j)
        {
            Scalar value = area * ((dx2(i) + dy2(i)) * (dx2(j) + dy2(j)) +
                                   (1-m_sigma) * ( 2*dxy(i)*dxy(j) - dx2(i)*dy2(j) - dy2(i)*dx2(j)));
            if((i < 3 || orient[i%3]) != (j < 3 || orient[j%3]))
            {
                value *= -1;
            }
            *(it++) = Triplet(nodes[i], nodes[j], value);
        }
    }
}


}
