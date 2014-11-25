
#ifndef VGMESH_H
#define VGMESH_H


#include <cassert>
#include <climits>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "../../common/surface_mesh/surfaceMesh.h"


namespace Vitelotte
{

template < typename _Scalar, int _Dim=2, int _Chan=4 >
class VGMesh: public Patate::SurfaceMesh
{
public:
    typedef _Scalar Scalar;

    enum {
        Dim = _Dim,
        Chan = _Chan
    };

    typedef VGMesh<Scalar, Dim, Chan> Self;

    typedef Eigen::Matrix<Scalar, Dim, 1> Vector;
    typedef Eigen::Matrix<Scalar, Chan, 1> NodeValue;

    typedef std::vector<NodeValue> NodeVector;

    struct Node : public BaseHandle
    {
        explicit Node(int _idx = -1) : BaseHandle(_idx) {}
        std::ostream& operator<<(std::ostream& os) const { return os << 'n' << idx(); }
    };

    static const NodeValue UnconstrainedNode;

    enum {
        // Nodes
        VertexValue         = 0x01,
        VertexFromValue     = 0x02,
        EdgeValue           = 0x04,
        EdgeGradient        = 0x08,

        // Specials
//        VertexGradientSpecial = 0x10,

        // Aggregates
        Linear = VertexValue | VertexFromValue,
        Quadratic = Linear | EdgeValue,

        Morley = Linear | EdgeGradient /*| VertexGradientSpecial*/,
        FV = Quadratic | EdgeGradient /*| VertexGradientSpecial*/
    };


public:
    explicit VGMesh(unsigned attributes = 0);
    virtual ~VGMesh() {}

    VGMesh(const Self& other)
    { operator=(other); }

    VGMesh& operator=(const Self& rhs);

    unsigned getAttributes() const { return m_attributes; }
    void setAttributes(unsigned attributes);

    bool hasVertexValue() const { return m_attributes & VertexValue; }
    bool hasVertexFromValue() const { return m_attributes & VertexFromValue; }
    bool hasEdgeValue() const { return m_attributes & EdgeValue; }
    bool hasEdgeGradient() const { return m_attributes & EdgeGradient; }
//    bool hasVertexGradientSpecial() const { return m_attributes & VertexGradientSpecial; }


    bool hasEdgeConstraintFlag() const { return bool(m_edgeConstraintFlag); }
    void setEdgeConstraintFlag(bool on);
    Patate::Property<bool>::Reference isEdgeConstrained(Edge edge)
        { return m_edgeConstraintFlag[edge]; }
    Patate::Property<bool>::ConstReference isEdgeConstrained(Edge edge) const
        { return m_edgeConstraintFlag[edge]; }


public: //--- Nodes -----------------------------------------------------------

    inline typename NodeVector::size_type nNodes() const
    { return m_nodes.size(); }

    inline bool isConstraint(Node node) const
    { return !isnan(nodeValue(node)[0]); }

    inline const NodeValue& nodeValue(Node node) const
    { return m_nodes.at(node.idx()); }

    inline NodeValue& nodeValue(Node node) { return m_nodes.at(node.idx()); }

    inline Node addNode(const NodeValue& nodeValue=UnconstrainedNode);

    void compactNodes();

public: //--- Constraints edition ---------------------------------------------

//    void setValueConstraint(Halfedge h, Node from, Node mid, Node to);
//    void setContinuousValueConstraint(Halfedge h,
//                                      Node from, Node mid, Node to)
//        { setValueConstraint(h, from, mid, to);
//          setValueConstraint(oppositeHalfedge(h), to, mid, from); }

//    bool isValueConstraint(Edge e) const;

    void setVertexNode(Node node, Halfedge from, Halfedge to);
    void setSingularity(Node fromNode, Node toNode, Halfedge from, Halfedge to);

    void finalize();


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

    inline bool isValid(Node n) const;

    inline int halfedgeOrientation(Halfedge h) const {
        // It is possible to use h.idx() % 2 for a similar result, but it makes
        // the halfedge orientation hard to predict.
        return fromVertex(h).idx() > toVertex(h).idx();
    }


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

    inline const Vector& position(Vertex v) const { return m_positions[v]; }
    inline Vector& position(Vertex v) { return m_positions[v]; }

    inline Node vertexValueNode(Halfedge h) const
        { assert(m_attributes | VertexValue); return m_vertexValueNodes[h]; }
    inline Node& vertexValueNode(Halfedge h)
        { assert(m_attributes | VertexValue); return m_vertexValueNodes[h]; }

    inline Node vertexFromValueNode(Halfedge h) const
        { assert(m_attributes | VertexFromValue); return m_vertexFromValueNodes[h]; }
    inline Node& vertexFromValueNode(Halfedge h)
        { assert(m_attributes | VertexFromValue); return m_vertexFromValueNodes[h]; }

    inline Node edgeValueNode(Halfedge h) const
        { assert(m_attributes | EdgeValue); return m_edgeValueNodes[h]; }
    inline Node& edgeValueNode(Halfedge h)
        { assert(m_attributes | EdgeValue); return m_edgeValueNodes[h]; }

//    inline Node vertexGradientNode(Vertex v) const
//        { assert(m_attributes | VertexGradient); return m_vertexGradientNodes[v]; }
//    inline Node& vertexGradientNode(Vertex v)
//        { assert(m_attributes | VertexGradient); return m_vertexGradientNodes[v]; }

    inline Node edgeGradientNode(Halfedge h) const
        { assert(m_attributes | EdgeGradient); return m_edgeGradientNodes[h]; }
    inline Node& edgeGradientNode(Halfedge h)
        { assert(m_attributes | EdgeGradient); return m_edgeGradientNodes[h]; }

    inline VertexProperty<Vector>& positionProperty() { return m_positions; }
    inline const VertexProperty<Vector>& positionProperty() const
        { return m_positions; }


protected:
    NodeVector m_nodes;

    VertexProperty<Vector> m_positions;

    unsigned m_attributes;
    HalfedgeProperty<Node> m_vertexValueNodes;
    HalfedgeProperty<Node> m_vertexFromValueNodes;
    HalfedgeProperty<Node> m_edgeValueNodes;
//    HalfedgeProperty<Node> m_vertexGradientSpecial;
    HalfedgeProperty<Node> m_edgeGradientNodes;

    EdgeProperty<bool> m_edgeConstraintFlag;
};


} // namespace Vitelotte

#include "vgMesh.hpp"


#endif // VGMESH_H

