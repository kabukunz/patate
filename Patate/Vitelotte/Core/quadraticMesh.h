
#ifndef QUADRATICMESH_H
#define QUADRATICMESH_H


#include <cassert>
#include <climits>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "../../common/surface_mesh/surfaceMesh.h"


namespace Vitelotte
{

namespace internal
{

template < typename Mesh >
void compactNodes(Mesh& mesh);

}


template < typename _Scalar, int _Dim=2, int _Chan=4 >
class QuadraticMesh: public Patate::SurfaceMesh
{
public:
    typedef _Scalar Scalar;

    enum {
        Dim = _Dim,
        Chan = _Chan
    };

    typedef QuadraticMesh<Scalar, Dim, Chan> Self;

    typedef Eigen::Matrix<Scalar, Dim, 1> Vector;
    typedef Eigen::Matrix<Scalar, Chan, 1> NodeValue;

    typedef std::vector<NodeValue> NodeVector;
    typedef unsigned NodeID;
    //typedef typename NodeVector::size_type NodeID;

    enum {
        InvalidNodeID = UINT_MAX // std::numeric_limits<NodeID>::max()
    };

    static const NodeValue UnconstrainedNode;


public:
    QuadraticMesh();
    virtual ~QuadraticMesh() {}

    template < typename OtherScalar >
    QuadraticMesh(const QuadraticMesh<OtherScalar, _Dim, _Chan>& other)
    { operator=(other); }

    template < typename OtherScalar >
    QuadraticMesh& operator=(const QuadraticMesh<OtherScalar, _Dim, _Chan>& rhs);


public: //--- Nodes -----------------------------------------------------------

    inline typename NodeVector::size_type nNodes() const
    { return m_nodes.size(); }

    inline bool isConstraint(NodeID node) const
    { return !isnan(nodeValue(node)[0]); }

    inline const NodeValue& nodeValue(NodeID node) const
    { return m_nodes.at(node); }

    inline NodeValue& nodeValue(NodeID node) { return m_nodes.at(node); }

    inline NodeID addNode(const NodeValue& nodeValue=UnconstrainedNode);

    void compactNodes();

protected:
    template < typename Marked >
    void markNodes(Halfedge h, Marked& marked) const;

    template < typename Map >
    void remapNodes(Halfedge h, Map& map);

public: //--- Constraints edition ---------------------------------------------

    void initializeValueConstraints();

    void setValueConstraint(Halfedge h, NodeID from, NodeID mid, NodeID to);
    void setContinuousValueConstraint(Halfedge h,
                                      NodeID from, NodeID mid, NodeID to)
        { setValueConstraint(h, from, mid, to);
          setValueConstraint(oppositeHalfedge(h), to, mid, from); }

    bool isValueConstraint(Edge e) const;

    void propagateConstraints();

protected:
    void processSingularity(HalfedgeAroundVertexCirculator& hit, Halfedge end);


public: //--- Quadratic patches -----------------------------------------------

    inline bool isSingular(Halfedge h) const;
    inline bool isSingular(Face f) const;

    inline unsigned nSingularFaces() const;


public: //--- Utility ---------------------------------------------------------

    inline void reserve(unsigned nvertices, unsigned nedges, unsigned nfaces,
                        unsigned nnodes);
    inline void clear();

    inline Vertex addVertex(const Vector& pos);

    inline bool isValid(Vertex v) const { return Patate::SurfaceMesh::isValid(v); }
    inline bool isValid(Halfedge h) const { return Patate::SurfaceMesh::isValid(h); }
    inline bool isValid(Edge e) const { return Patate::SurfaceMesh::isValid(e); }
    inline bool isValid(Face f) const { return Patate::SurfaceMesh::isValid(f); }

    inline bool isValid(NodeID n) const;


protected: //--- Topological operations ---------------------------------------

    inline void triangulate() { assert(false); }
    inline void triangulate(Face /*f*/) { assert(false); }

    inline bool isCollapseOk(Halfedge h) { assert(false); return false; }
    inline void collapse(Halfedge h) { assert(false); }

    inline void split(Face f, Vertex v) { assert(false); }
    inline void split(Edge e, Vertex v) { assert(false); }

    inline Halfedge insertVertex(Edge e, Vertex v)
        { assert(false); return Halfedge(); }
    inline Halfedge insertVertex(Halfedge h, Vertex v)
        { assert(false); return Halfedge(); }

    inline Halfedge insertEdge(Halfedge h0, Halfedge h1)
        { assert(false); return Halfedge(); }

    inline bool isFlipOk(Edge e) const { assert(false); return false; }
    inline void flip(Edge e) { assert(false); }


public: //--- Attributes accessors --------------------------------------------

    inline const Vector& position(Vertex v) const { return m_vPos[v]; }
    inline Vector& position(Vertex v) { return m_vPos[v]; }

    inline NodeID fromNode(Halfedge h) const { return m_hFromNode[h]; }
    inline NodeID& fromNode(Halfedge h) { return m_hFromNode[h]; }

    inline NodeID toNode(Halfedge h) const { return m_hToNode[h]; }
    inline NodeID& toNode(Halfedge h) { return m_hToNode[h]; }

    inline NodeID midNode(Halfedge h) const { return m_hMidNode[h]; }
    inline NodeID& midNode(Halfedge h) { return m_hMidNode[h]; }

    inline VertexProperty<Vector>& vPosProperty() { return m_vPos; }
    inline const VertexProperty<Vector>& vPosProperty() const
    { return m_vPos; }


protected:
    NodeVector m_nodes;

    VertexProperty<Vector> m_vPos;

    HalfedgeProperty<NodeID> m_hFromNode;
    HalfedgeProperty<NodeID> m_hToNode;
    HalfedgeProperty<NodeID> m_hMidNode;

    friend void internal::compactNodes<Self>(Self&);
};


#include "quadraticMesh.hpp"

} // namespace Vitelotte

#endif // QUADRATICMESH_H

