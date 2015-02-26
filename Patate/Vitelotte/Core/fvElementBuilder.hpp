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
    return mesh.nVertexGradientConstraints(element)? 117: 81;
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
void
FVElementBuilder<_Mesh, _Scalar>::
        setRhs(const Mesh& mesh, IndexMap imap, Matrix& rhs) {

    rhs.setZero();

    for(typename Mesh::HalfedgeIterator hit = mesh.halfedgesBegin();
        hit != mesh.halfedgesEnd(); ++hit) {

        if(mesh.nVertexGradientConstraints(*hit) == 0)
            continue;

        typename Mesh::Vertex from = mesh.fromVertex(*hit);
        typename Mesh::Vertex to   = mesh.  toVertex(*hit);
        typename Mesh::Node n = mesh.vertexGradientDummyNode(*hit);
        if(n.isValid()) {
            bool v0c = mesh.isGradientConstraint(from);
            const typename Mesh::Gradient& grad = mesh.gradientConstraint(v0c? from: to);
            typename Mesh::Vector v = mesh.position(to) - mesh.position(from);
            if(!v0c) v = -v;
            typename Mesh::NodeValue cons = grad * v;
            rhs.row(imap(n.idx())) = cons.template cast<Scalar>();
        }
    }
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
    do ++hit;
    while(!mesh.isGradientConstraint(mesh.toVertex(*hit)) && hit != hend);
    bool isPgc = mesh.isGradientConstraint(mesh.toVertex(*hit));
    typename Mesh::Edge e2 = mesh.edge(*hit);
    typename Mesh::Halfedge h2 = *hit;
    ++hit;
    typename Mesh::Edge e1 = mesh.edge(*hit);
    typename Mesh::Halfedge h1 = *hit;
    --hit;
//    if(isPgc) std::cout << "pgc2: " << element.idx() << "\n";

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
        typename Elem::Hessian hessians[9];
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

    if(isPgc)
    {
//        std::cout << "Flat elem:\n";
//        std::cout << "  p0: " << elem.point(0).transpose() << "\n";
//        std::cout << "  p1: " << elem.point(1).transpose() << "\n";
//        std::cout << "  p2: " << elem.point(2).transpose() << "\n";
//        std::cout << "  n8: " << mesh.nodeValue(typename Mesh::Node(nodes[8])).transpose() << "\n";
//        std::cout << "  n7: " << mesh.nodeValue(typename Mesh::Node(nodes[7])).transpose() << "\n";
//        std::cout << "  Stiffness matrix:\n" << sm << "\n";

        typedef Eigen::Matrix<Scalar, 9, 1> Vector9;
        Vector9 fde1, fde2;
        fde1 <<
            -1.0L/2.0L*(elem.doubleArea()*(2*elem.dldn(0, 1) + elem.dldn(1, 1)) + 7*elem.edgeLength(1))/(elem.edgeLength(1)*elem.edgeLength(2)),
            (1.0L/2.0L)*(elem.doubleArea()*(elem.dldn(0, 0) + 2*elem.dldn(1, 0)) - elem.edgeLength(0))/(elem.edgeLength(0)*elem.edgeLength(2)),
            -1.0L/2.0L*elem.doubleArea()*(elem.edgeLength(0)*(elem.dldn(1, 1) + 2*elem.dldn(2, 1)) - elem.edgeLength(1)*(elem.dldn(0, 0) + 2*elem.dldn(2, 0)))/(elem.edgeLength(0)*elem.edgeLength(1)*elem.edgeLength(2)),
            -4/elem.edgeLength(2),
            4/elem.edgeLength(2),
            4/elem.edgeLength(2),
            elem.doubleArea()/(elem.edgeLength(0)*elem.edgeLength(2)),
            -elem.doubleArea()/(elem.edgeLength(1)*elem.edgeLength(2)),
            0;
        fde2 <<
            -1.0L/2.0L*(elem.doubleArea()*(2*elem.dldn(0, 2) + elem.dldn(2, 2)) + 7*elem.edgeLength(2))/(elem.edgeLength(1)*elem.edgeLength(2)),
            -1.0L/2.0L*elem.doubleArea()*(elem.edgeLength(0)*(2*elem.dldn(1, 2) + elem.dldn(2, 2)) - elem.edgeLength(2)*(elem.dldn(0, 0) + 2*elem.dldn(1, 0)))/(elem.edgeLength(0)*elem.edgeLength(1)*elem.edgeLength(2)),
            (1.0L/2.0L)*(elem.doubleArea()*(elem.dldn(0, 0) + 2*elem.dldn(2, 0)) - elem.edgeLength(0))/(elem.edgeLength(0)*elem.edgeLength(1)),
            -4/elem.edgeLength(1),
            4/elem.edgeLength(1),
            4/elem.edgeLength(1),
            elem.doubleArea()/(elem.edgeLength(0)*elem.edgeLength(1)),
            0,
            -elem.doubleArea()/(elem.edgeLength(1)*elem.edgeLength(2));

        int ce1 = mesh.vertexGradientDummyNode(h1).idx();
        int ce2 = mesh.vertexGradientDummyNode(h2).idx();
        if(ce1 < 0 || ce2 < 0)
        {
            error(STATUS_ERROR, "Invalid node");
            return;
        }
        for(size_t i = 0; i < 9; ++i)
        {
            Scalar f = (i < 6 || orient[i%3])? 1: -1;
            *(it++) = Triplet(nodes[i], ce1, fde1(i) * f);
            *(it++) = Triplet(ce1, nodes[i], fde1(i) * f);
            *(it++) = Triplet(nodes[i], ce2, fde2(i) * f);
            *(it++) = Triplet(ce2, nodes[i], fde2(i) * f);
        }
    }
}


}
