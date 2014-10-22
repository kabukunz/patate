
#ifndef FVMESH_H
#define FVMESH_H


#include <cassert>
#include <climits>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "quadraticMesh.h"


namespace Vitelotte
{


template < typename _Scalar, int _Dim=2, int _Chan=4 >
class FVMesh: public QuadraticMesh<_Scalar, _Dim, _Chan>
{
public:
    typedef QuadraticMesh<_Scalar, _Dim, _Chan> Base;
    typedef FVMesh<_Scalar, _Dim, _Chan> Self;

    typedef typename Base::Scalar Scalar;

    typedef typename Base::Vertex Vertex;
    typedef typename Base::Halfedge Halfedge;
    typedef typename Base::Edge Edge;
    typedef typename Base::Face Face;

//    typedef typename Base::VertexProperty VertexProperty;
//    typedef typename Base::HalfedgeProperty HalfedgeProperty;
//    typedef typename Base::EdgeProperty EdgeProperty;
//    typedef typename Base::FaceProperty FaceProperty;

    typedef typename Base::VertexIterator VertexIterator;
    typedef typename Base::HalfedgeIterator HalfedgeIterator;
    typedef typename Base::EdgeIterator EdgeIterator;
    typedef typename Base::FaceIterator FaceIterator;

    typedef typename Base::VertexAroundVertexCirculator VertexAroundVertexCirculator;
    typedef typename Base::HalfedgeAroundVertexCirculator HalfedgeAroundVertexCirculator;
    typedef typename Base::FaceAroundVertexCirculator FaceAroundVertexCirculator;
    typedef typename Base::VertexAroundFaceCirculator VertexAroundFaceCirculator;
    typedef typename Base::HalfedgeAroundFaceCirculator HalfedgeAroundFaceCirculator;

    enum {
        Dim = Base::Dim,
        Chan = Base::Chan
    };

    typedef typename Base::Vector Vector;
    typedef typename Base::NodeValue NodeValue;

    typedef typename Base::NodeVector NodeVector;
    typedef typename Base::NodeID NodeID;

    enum {
        InvalidNodeID = Base::InvalidNodeID
    };

public:
    FVMesh();
    virtual ~FVMesh() {}

    template < typename OtherScalar >
    FVMesh(const FVMesh<OtherScalar, _Dim, _Chan>& other)
    { operator=(other); }

    template < typename OtherScalar >
    FVMesh& operator=(const FVMesh<OtherScalar, _Dim, _Chan>& rhs);


    void compactNodes();

protected:
    template < typename Marked >
    void markNodes(Halfedge h, Marked& marked) const;

    template < typename Map >
    void remapNodes(Halfedge h, Map& map);


public: //--- Attributes accessors --------------------------------------------

    inline bool hasFlatGradient(Vertex v) const { return m_vFlatGrad[v]; }
    inline void setFlatGradient(Vertex v, bool flat) { m_vFlatGrad[v] = flat; }

    inline NodeID gradientNode(Halfedge h) const { return m_hGradNode[h]; }
    inline NodeID& gradientNode(Halfedge h) { return m_hGradNode[h]; }


protected:
    Patate::SurfaceMesh::VertexProperty<bool> m_vFlatGrad;

    Patate::SurfaceMesh::HalfedgeProperty<NodeID> m_hGradNode;

    friend void internal::compactNodes<Self>(Self&);
};


#include "fvMesh.hpp"

} // namespace Vitelotte

#endif // FVMESH_H

