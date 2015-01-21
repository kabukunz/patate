/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _VITELOTTE_VG_MESH_
#define _VITELOTTE_VG_MESH_


#include <cassert>
#include <climits>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "../../common/surface_mesh/surfaceMesh.h"


namespace Vitelotte
{

/**
 * \brief A mesh with data suitable for representing complex color gradients,
 * among other.
 *
 * \note There is currently no simple way to convert a `VGMesh` of one kind
 * to an other of an other kind (different `Scalar` type, differend dimention,
 * etc.)
 *
 *
 * \note There is currently no way to suppress nodes, except by using
 * compactNodes() method. The rational behind this is that it would be unsafe
 * because some halfedge may reference it. As we don't have conectivity
 * information for the nodes and that it would be too expansive to go through
 * all halfedge each time we remove a node, it is better that way.
 *
 *
 * \note We may add connectivity information for nodes, but it would be quite
 * expansive in terms of memory and not very usefull. We may add it later
 * and make it optional.
 */
template < typename _Scalar, int _Dim=2, int _Chan=4 >
class VGMesh: public PatateCommon::SurfaceMesh
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

    /// \brief A special NodeValue used for unconstrained nodes.
    static const NodeValue UnconstrainedNode;

    enum HalfedgeAttribute{
        TO_VERTEX_VALUE,
        FROM_VERTEX_VALUE,
        EDGE_VALUE,
        EDGE_GRADIENT,
    };

    enum {
        // Nodes
        TO_VERTEX_VALUE_FLAG    = (1 << TO_VERTEX_VALUE),
        FROM_VERTEX_VALUE_FLAG  = (1 << FROM_VERTEX_VALUE),
        EDGE_VALUE_FLAG         = (1 << EDGE_VALUE),
        EDGE_GRADIENT_FLAG      = (1 << EDGE_GRADIENT),

        // Specials
//        VertexGradientConstraint = 0x10,

        // Aggregates
        LINEAR_FLAGS = TO_VERTEX_VALUE_FLAG | FROM_VERTEX_VALUE_FLAG,
        QUADRATIC_FLAGS = LINEAR_FLAGS | EDGE_VALUE_FLAG,

        MORLEY_FLAGS = LINEAR_FLAGS | EDGE_GRADIENT_FLAG /*| VertexGradientSpecial*/,
        FV_FLAGS = QUADRATIC_FLAGS | EDGE_GRADIENT_FLAG /*| VertexGradientSpecial*/
    };


public:

    /// \name Constructor, derstructor, assignement
    /// \{

    /// \brief Create a VGMesh with `attirbutes` flags activated.
    explicit VGMesh(unsigned attributes = 0);
    virtual ~VGMesh() {}

    VGMesh(const Self& other)
    { operator=(other); }

    VGMesh& operator=(const Self& rhs);

    /// \}


    /// \name Mesh and general
    /// \{

    inline void reserve(unsigned nvertices, unsigned nedges, unsigned nfaces,
                        unsigned nnodes);
    inline void clear();

    inline Vertex addVertex(const Vector& pos);

    inline const Vector& position(Vertex v) const { return m_positions[v]; }
    inline Vector& position(Vertex v) { return m_positions[v]; }

//    inline VertexProperty<Vector>& positionProperty() { return m_positions; }
//    inline const VertexProperty<Vector>& positionProperty() const
//        { return m_positions; }

    inline bool isValid(Vertex v) const { return PatateCommon::SurfaceMesh::isValid(v); }
    inline bool isValid(Halfedge h) const { return PatateCommon::SurfaceMesh::isValid(h); }
    inline bool isValid(Edge e) const { return PatateCommon::SurfaceMesh::isValid(e); }
    inline bool isValid(Face f) const { return PatateCommon::SurfaceMesh::isValid(f); }

    inline bool isValid(Node n) const;

    /**
     * \brief Return a boolean such that opposite halfedges give a different
     * result.
     *
     * In practice, return `fromVertex(h).idx() > toVertex(h).idx()`. This
     * make the method easily predictable.
     */
    inline bool halfedgeOrientation(Halfedge h) const {
        // It is possible to use h.idx() % 2 for a similar result, but it makes
        // the halfedge orientation hard to predict.
        return fromVertex(h).idx() > toVertex(h).idx();
    }


    /// \}


    /// \name Attributes
    /// \{

    /// \brief Return active attribute flags ored together.
    unsigned getAttributes() const { return m_attributes; }

    /**
     * \brief Set the active attributes to `attributes`.
     *
     * Attributes that are removed will release corresponding memory and
     * new attributes will be set to their default value. Attibute that stay
     * active won't be modified.
     *
     * \param attributes Attributes flags ORed together.
     */
    void setAttributes(unsigned attributes);

    inline bool hasToVertexValue() const { return m_attributes & TO_VERTEX_VALUE_FLAG; }
    inline bool hasFromVertexValue() const { return m_attributes & FROM_VERTEX_VALUE_FLAG; }
    inline bool hasEdgeValue() const { return m_attributes & EDGE_VALUE_FLAG; }
    inline bool hasEdgeGradient() const { return m_attributes & EDGE_GRADIENT_FLAG; }
//    bool hasVertexGradientSpecial() const { return m_attributes & VertexGradientSpecial; }


//    bool hasEdgeConstraintFlag() const { return bool(m_edgeConstraintFlag); }
//    void setEdgeConstraintFlag(bool on);
//    PatateCommon::Property<bool>::Reference isEdgeConstrained(Edge edge)
//        { return m_edgeConstraintFlag[edge]; }
//    PatateCommon::Property<bool>::ConstReference isEdgeConstrained(Edge edge) const
//        { return m_edgeConstraintFlag[edge]; }

    inline Node toVertexValueNode(Halfedge h) const
        { assert(hasToVertexValue()); return m_toVertexValueNodes[h]; }
    inline Node& toVertexValueNode(Halfedge h)
        { assert(hasToVertexValue()); return m_toVertexValueNodes[h]; }

    inline Node fromVertexValueNode(Halfedge h) const
        { assert(hasFromVertexValue()); return m_fromVertexValueNodes[h]; }
    inline Node& fromVertexValueNode(Halfedge h)
        { assert(hasFromVertexValue()); return m_fromVertexValueNodes[h]; }

    inline Node edgeValueNode(Halfedge h) const
        { assert(hasEdgeValue()); return m_edgeValueNodes[h]; }
    inline Node& edgeValueNode(Halfedge h)
        { assert(hasEdgeValue()); return m_edgeValueNodes[h]; }

//    inline Node vertexGradientNode(Vertex v) const
//        { assert(m_attributes | VertexGradient); return m_vertexGradientNodes[v]; }
//    inline Node& vertexGradientNode(Vertex v)
//        { assert(m_attributes | VertexGradient); return m_vertexGradientNodes[v]; }

    inline Node edgeGradientNode(Halfedge h) const
        { assert(hasEdgeGradient()); return m_edgeGradientNodes[h]; }
    inline Node& edgeGradientNode(Halfedge h)
        { assert(hasEdgeGradient()); return m_edgeGradientNodes[h]; }

    HalfedgeAttribute oppositeAttribute(HalfedgeAttribute attr) const;

    inline Node halfedgeNode(Halfedge h, HalfedgeAttribute attr) const;
    inline Node& halfedgeNode(Halfedge h, HalfedgeAttribute attr);

    inline Node halfedgeOppositeNode(Halfedge h, HalfedgeAttribute attr) const;
    inline Node& halfedgeOppositeNode(Halfedge h, HalfedgeAttribute attr);

    /// \}


    /// \name Low-level nodes manipulation
    /// \{

    /// \brief Return the number of nodes.
    inline typename NodeVector::size_type nNodes() const
    { return m_nodes.size(); }

    /**
     * \brief Add and return a node with value `nodeValue` or an unknown node
     * if no parameter is provided.
     */
    inline Node addNode(const NodeValue& nodeValue=UnconstrainedNode);

    /// \brief Read only access to the value of `node`.
    inline const NodeValue& nodeValue(Node node) const
    { return m_nodes.at(node.idx()); }

    /// \brief Read-write access to the value of `node`.
    inline NodeValue& nodeValue(Node node) { return m_nodes.at(node.idx()); }

    /// \brief Return true iff `node` is a constraint.
    inline bool isConstraint(Node node) const
    { return isValid(node) && !isnan(nodeValue(node)[0]); }

    /**
     * \brief Return true if the mesh has unknown nodes.
     * \par Complexity
     * This function operates in \f$O(n)\f$ with \f$n\f$ the number of nodes.
     */
    inline bool hasUnknowns() const;

    /// \}


    /// \name High-level nodes manipulation
    /// \{

//    void setValueConstraint(Halfedge h, Node from, Node mid, Node to);
//    void setContinuousValueConstraint(Halfedge h,
//                                      Node from, Node mid, Node to)
//        { setValueConstraint(h, from, mid, to);
//          setValueConstraint(oppositeHalfedge(h), to, mid, from); }

//    bool isValueConstraint(Edge e) const;

    /**
     * \brief Set nodes of all the halfedges inbetween halfedges `from` and
     * `to` (in counter-clockwise direction) to `node`.
     *
     * \pre `from` and `to` mush have the same source vertex.
     */
    void setVertexNode(Node node, Halfedge from, Halfedge to);

    /**
     * \brief Similar to VGMesh::setVertexNode, but for singularities.
     *
     * set nodes for a singularity starting with value `fromNode` in the
     * direction of halfedge `from` to value `toNode` in the direction of
     * halfedge `to`.
     *
     * \pre The mesh must have the VertexFromValue attribute active.
     * \pre `from` and `to` mush have the same source vertex.
     * \pre `fromNode` and `toNode` mush be constraints.
     */
    void setSingularity(Node fromNode, Node toNode, Halfedge from, Halfedge to);

//    void removeUselessUnknownNodes();

    /**
     * \brief Simplify constraints when possible.
     *
     * This method operate in two steps:
     * 1. Simplify each pair of opposite nodes (see
     *    VGMesh::simplifyOppositeNodes()).
     * 2. Go around each vertex and try to simplfy them.
     *
     * After processing, the resulting mesh will use less nodes than the
     * original if simplifications where possible, but still produce the same
     * final result after finalization and solving. It may be usefull to make
     * the solver slightly faster or to diminish file size.
     *
     * One typically which to call VGMesh::compactNodes() after this function
     * to effectively remove now unused nodes.
     *
     * \note Currently, this method only work locally. This mean for instance
     * that it won't merge all constraint nodes with the same value if they are
     * not on the same primitive.
     *
     * \warning This method does not support non-local unknown node, ie.
     * unknown node used on different primitive of the mesh to achieve special
     * effects like color "teleportation". Using it in such case may lead to
     * unexpected results.
     */
    void simplifyConstraints();

    void simplifyVertexArcConstraints(Halfedge from, Halfedge to);
    void simplifyEdgeConstraints(Edge e);

    /**
     * \brief Replace `n0` and `n1` by a single node on an invalid node if
     * possible.
     *
     * This is mainly an helper funciton for VGMesh::simplifyConstraints().
     *
     * \warning This method does not support non-local unknown node, ie.
     * unknown node used on different primitive of the mesh to achieve special
     * effects like color "teleportation". Using it in such case may lead to
     * unexpected results.
     */
    void simplifyOppositeNodes(Node& n0, Node& n1) const;

    /**
     * /brief Finalize a mesh with invalid node so that it can be send to a
     * Solver.
     *
     * \todo create a tutorial page about this an link it from here.
     *
     * \warning This method does not support non-local unknown node, ie.
     * unknown node used on different primitive of the mesh to achieve special
     * effects like color "teleportation". Using it in such case may lead to
     * unexpected results.
     */
    void finalize();

    void finalizeVertexArc(Halfedge from, Halfedge to);
    void finalizeEdge(Edge e);

    /**
     * \brief Remove unused nodes and reassign node indices accordingly.
     *
     * \warning This method invalidate all Node handles. There is currently no
     * way access old-to-new node mapping.
     */
    void compactNodes();

    /// \}
    ///


    /// \name Quadratic patches
    /// \{

    /// \brief Return true if the target vertex of `h` is singular.
    inline bool isSingular(Halfedge h) const;

    /// \brief Return the number of singular vertex around `f`.
    inline unsigned nSingulars(Face f) const;

    /**
     * \brief Return the number of singular faces in the mesh.
     *
     * \par Complexity
     * This method opperates in \f$O(n)\f$ with \f$n\f$ the number of
     * halfedges.
     */
    inline unsigned nSingularFaces() const;

    /// \}


protected:
    /// \name Removed topological operations
    /// \{

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

    /// \}

    /// \name Helper methods
    /// \{

    void findConstrainedEdgesSimplify(Vertex vx,
                                      std::vector<Halfedge>& consEdges);

    /// \}


protected:
    NodeVector m_nodes;

    unsigned m_attributes;

    VertexProperty<Vector> m_positions;
//    VertexProperty<bool> m_vertexGradientConstrained;

    HalfedgeProperty<Node> m_toVertexValueNodes;
    HalfedgeProperty<Node> m_fromVertexValueNodes;
    HalfedgeProperty<Node> m_edgeValueNodes;
    HalfedgeProperty<Node> m_edgeGradientNodes;

    EdgeProperty<bool> m_edgeConstraintFlag;
};


} // namespace Vitelotte

#include "vgMesh.hpp"


#endif // VGMESH_H

