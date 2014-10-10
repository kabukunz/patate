
#ifndef FVMESH_H
#define FVMESH_H


#include <cassert>
#include <climits>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "../../common/surface_mesh/surfaceMesh.h"


namespace Vitelotte
{


template < typename _Scalar, int _Dim=2, int _Chan=4 >
class FVMesh: public Patate::SurfaceMesh
{
public:
    typedef _Scalar Scalar;

    enum {
        Dim = _Dim,
        Chan = _Chan
    };

    typedef Eigen::Matrix<Scalar, Dim, 1> Vector;
    typedef Eigen::Matrix<Scalar, Chan, 1> NodeValue;

    typedef std::vector<NodeValue> NodeVector;
    typedef unsigned NodeID;
    //typedef typename NodeVector::size_type NodeID;

    enum {
        InvalidNodeID = UINT_MAX // std::numeric_limits<NodeID>::max()
    };

    static const NodeValue UnconstrainedNode;

    struct NodeCompare
    {
        inline NodeCompare(const FVMesh& em) : em(em) {}
        inline bool operator()(NodeID lhs, NodeID rhs)
        {
            bool lCons = em.isConstraint(lhs);
            bool rCons = em.isConstraint(rhs);
            return !lCons && rCons;
        }

    private:
        const FVMesh& em;
    };

public:
    FVMesh();
    virtual ~FVMesh() {}

    template < typename OtherScalar >
    FVMesh(const FVMesh<OtherScalar, _Dim, _Chan>& other)
    { operator=(other); }

    template < typename OtherScalar >
    FVMesh& operator=(const FVMesh<OtherScalar, _Dim, _Chan>& rhs);


public: //--- Nodes -----------------------------------------------------------

    inline typename NodeVector::size_type nNodes() const
    { return m_nodes.size(); }

    inline bool isConstraint(NodeID node) const
    { return !isnan(nodeValue(node)[0]); }

    inline const NodeValue& nodeValue(NodeID node) const
    { return m_nodes.at(node); }

    inline NodeValue& nodeValue(NodeID node) { return m_nodes.at(node); }

    inline NodeID addNode(const NodeValue& nodeValue=UnconstrainedNode);

    /**
     * \brief Sort node so that unconstrained nodes have lower IDs than
     * constrained ones, and remove unused nodes.
     *
     * \warning As it reorder nodes, all previous NodeID are invalidated.
     */
    void sortAndCompactNodes();


public: //--- Utility ---------------------------------------------------------

    inline void reserve(unsigned nvertices, unsigned nedges, unsigned nfaces,
                        unsigned nnodes);
    inline Vertex addVertex(const Vector& pos);


protected: //--- Topological operations ---------------------------------------

    inline void triangulate() { assert(false); }
    inline void triangulate(Face /*f*/) { assert(false); }

    inline bool isCollapseOk(Halfedge h) { assert(false); }
    inline void collapse(Halfedge h) { assert(false); }

    inline void split(Face f, Vertex v) { assert(false); }
    inline void split(Edge e, Vertex v) { assert(false); }

    inline Halfedge insertVertex(Edge e, Vertex v) { assert(false); }
    inline Halfedge insertVertex(Halfedge h, Vertex v) { assert(false); }

    inline Halfedge insertEdge(Halfedge h0, Halfedge h1) { assert(false); }

    inline bool isFlipOk(Edge e) const { assert(false); }
    inline void flip(Edge e) { assert(false); }


public: //--- Attributes accessors --------------------------------------------

    inline const Vector& position(Vertex v) const { return m_vPos[v]; }
    inline Vector& position(Vertex v) { return m_vPos[v]; }

    inline bool hasFlatGradient(Vertex v) const { return m_vFlatGrad[v]; }
    inline void setFlatGradient(Vertex v, bool flat) { m_vFlatGrad[v] = flat; }

    inline NodeID fromNode(Halfedge h) const { return m_hFromNode[h]; }
    inline NodeID& fromNode(Halfedge h) { return m_hFromNode[h]; }

    inline NodeID toNode(Halfedge h) const { return m_hToNode[h]; }
    inline NodeID& toNode(Halfedge h) { return m_hToNode[h]; }

    inline NodeID midNode(Halfedge h) const { return m_hMidNode[h]; }
    inline NodeID& midNode(Halfedge h) { return m_hMidNode[h]; }

    inline NodeID gradientNode(Halfedge h) const { return m_hGradNode[h]; }
    inline NodeID& gradientNode(Halfedge h) { return m_hGradNode[h]; }


protected:
    NodeVector m_nodes;

    VertexProperty<Vector> m_vPos;
    VertexProperty<bool> m_vFlatGrad;

    HalfedgeProperty<NodeID> m_hFromNode;
    HalfedgeProperty<NodeID> m_hToNode;
    HalfedgeProperty<NodeID> m_hMidNode;
    HalfedgeProperty<NodeID> m_hGradNode;
};


#include "fvMesh.hpp"

} // namespace Vitelotte

#endif // FVMESH_H

