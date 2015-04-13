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
#include <map>

#include <Eigen/Core>

#include "../../common/surface_mesh/surfaceMesh.h"


namespace Vitelotte
{


enum
{
    Dynamic = Eigen::Dynamic
};


/**
 * \brief A mesh with data suitable for representing complex color gradients,
 * among other.
 *
 * Most generically, VGMesh represent a <em>n</em>-valued function over
 * a 2D surface embedded in a <em>d</em>-dimensional space. The number
 * of coefficients \e n and the number of dimension \e d are specified
 * by the template parameters \p _Coeffs and \p _Dims respectively. This
 * parameters can be set to the constant \c Dynamic if you want to be
 * able to set them at run time, or to a numeric constant for better
 * compile-time error checking and better performances.
 *
 * The surface is represented by a polygonal mesh. In practice, VGMesh
 * inherits from the class PatateCommon::SurfaceMesh, a customized
 * version of the
 * <a href="http://opensource.cit-ec.de/projects/surface_mesh">
 * Surface_mesh library</a>, which use an halfedge-based data structure.
 *
 * \section vitelotte_vg_mesh_attributes_nodes Attributes and Nodes
 *
 * The function value is defined per-face by some interpolation methods,
 * represented by the <tt>*Element</tt> classes
 * (see Concept::ElementConcept).
 *
 * Different interpolation methods requires different set of sample
 * points on each face. For instance, a linear interpolation
 * over a triangle only require values on the triangle vertices, where
 * quadratic interpolation require additional values at edge's midpoint.
 * The information stored to represent value samples is defined by the
 * mesh's \e attributes.
 *
 * Attributes are first set by the constructor and can be later
 * modified by calling setAttributes().
 *
 * \note There is currently no simple way to convert a `VGMesh` of one kind
 * to an other of an other kind (different `Scalar` type, different dimension,
 * etc.)
 *
 *
 * \note There is currently no way to delete nodes, except by using
 * deleteUnusedNodes() method. The rational behind this is that it would be
 * unsafe because some halfedge may reference it. As we don't have connectivity
 * information for the nodes and that it would be too expansive to go through
 * all halfedges each time we remove a node, it is better that way.
 *
 *
 * \note We may add connectivity information for nodes, but it would be quite
 * expansive in terms of memory and not very useful. We may add it later
 * and make it optional.
 */
template < typename _Scalar, int _Dims=2, int _Coeffs=4 >
class VGMesh: public PatateCommon::SurfaceMesh
{
public:
    typedef _Scalar Scalar;

    enum {
        DimsAtCompileTime = _Dims,
        CoeffsAtCompileTime = _Coeffs
    };

    typedef VGMesh<Scalar, DimsAtCompileTime, CoeffsAtCompileTime> Self;

    typedef Eigen::Matrix<Scalar, DimsAtCompileTime, 1> Vector;
    typedef Eigen::Matrix<Scalar, CoeffsAtCompileTime, 1> Value;
    typedef Eigen::Matrix<Scalar, CoeffsAtCompileTime, DimsAtCompileTime> Gradient;

protected:
    typedef Eigen::Matrix<Scalar, DimsAtCompileTime, Eigen::Dynamic> VectorMatrix;
    typedef Eigen::Matrix<Scalar, CoeffsAtCompileTime, Eigen::Dynamic> NodeMatrix;

public:
    typedef typename VectorMatrix::ColXpr VectorXpr;
    typedef typename VectorMatrix::ConstColXpr ConstVectorXpr;
    typedef typename NodeMatrix::ColXpr ValueXpr;
    typedef typename NodeMatrix::ConstColXpr ConstValueXpr;
    typedef typename Value::ConstantReturnType UnconstrainedNodeType;

    struct Node : public BaseHandle
    {
        explicit Node(int _idx = -1) : BaseHandle(_idx) {}
        std::ostream& operator<<(std::ostream& os) const { return os << 'n' << idx(); }
    };

    enum HalfedgeAttribute
    {
        TO_VERTEX_VALUE,
        FROM_VERTEX_VALUE,
        EDGE_VALUE,
        EDGE_GRADIENT,

        HALFEDGE_ATTRIB_COUNT
    };

    // TODO: Unused now. Use it or remove it ?
    enum VertexAttribute
    {
        VERTEX_POSITION,
        VERTEX_GRADIENT_CONSTRAINT,

        VERTEX_ATTRIB_COUNT
    };

    enum
    {
        // Nodes
        TO_VERTEX_VALUE_FLAG    = (1 << TO_VERTEX_VALUE),
        FROM_VERTEX_VALUE_FLAG  = (1 << FROM_VERTEX_VALUE),
        EDGE_VALUE_FLAG         = (1 << EDGE_VALUE),
        EDGE_GRADIENT_FLAG      = (1 << EDGE_GRADIENT),

        // Other
        VERTEX_GRADIENT_CONSTRAINT_FLAG = 0x100 | (1 << VERTEX_GRADIENT_CONSTRAINT),

        // Aggregates
        LINEAR_FLAGS = TO_VERTEX_VALUE_FLAG | FROM_VERTEX_VALUE_FLAG,
        QUADRATIC_FLAGS = LINEAR_FLAGS | EDGE_VALUE_FLAG,

        MORLEY_FLAGS = LINEAR_FLAGS | EDGE_GRADIENT_FLAG /*| VertexGradientSpecial*/,
        FV_FLAGS = QUADRATIC_FLAGS | EDGE_GRADIENT_FLAG | VERTEX_GRADIENT_CONSTRAINT_FLAG
    };


public:
    /// this class iterates linearly over all nodes
    /// \sa nodesBegin(), nodesEnd()
    class NodeIterator
    {
    public:

        /// Default constructor
        NodeIterator(Node n=Node(), const Self* m=NULL) : hnd_(n), mesh_(m)
        {
            if(mesh_ && mesh_->garbage())
            {
                while(mesh_->isValid(hnd_) && mesh_->isDeleted(hnd_))
                {
                    hnd_ = Node(hnd_.idx() + 1);
                }
            }
        }

        /// get the node the iterator refers to
        Node operator*()  const { return  hnd_; }

        /// are two iterators equal?
        bool operator==(const NodeIterator& rhs) const
        {
            return (hnd_==rhs.hnd_);
        }

        /// are two iterators different?
        bool operator!=(const NodeIterator& rhs) const
        {
            return !operator==(rhs);
        }

        /// pre-increment iterator
        NodeIterator& operator++()
        {
            hnd_ = Node(hnd_.idx() + 1);
            assert(mesh_);
            while(mesh_->garbage() && mesh_->isValid(hnd_) && mesh_->isDeleted(hnd_))
            {
                hnd_ = Node(hnd_.idx() + 1);
            }
            return *this;
        }

        /// pre-decrement iterator
        NodeIterator& operator--()
        {
            --hnd_.idx_;
            assert(mesh_);
            while (mesh_->garbage() && mesh_->isValid(hnd_) && mesh_->isDeleted(hnd_)) --hnd_.idx_;
            return *this;
        }

    private:
        Node  hnd_;
        const Self* mesh_;
    };

    /// Node property of type T
    /// \sa VertexProperty, HalfedgeProperty, EdgeProperty, FaceProperty
    template <class T> class NodeProperty : public PatateCommon::Property<T>
    {
    public:

        /// default constructor
        explicit NodeProperty() {}
        explicit NodeProperty(PatateCommon::Property<T> p)
            : PatateCommon::Property<T>(p) {}

        /// access the data stored for vertex \c v
        typename PatateCommon::Property<T>::Reference operator[](Node n)
        {
            return PatateCommon::Property<T>::operator[](n.idx());
        }

        /// access the data stored for vertex \c v
        typename PatateCommon::Property<T>::ConstReference operator[](Node n) const
        {
            return PatateCommon::Property<T>::operator[](n.idx());
        }
    };

public:

    /// \name Constructor, derstructor, assignement
    /// \{

    /// \brief Create a VGMesh with `attirbutes` flags activated.
    explicit VGMesh(unsigned attributes = 0);
    explicit VGMesh(unsigned nDims, unsigned nCoeffs, unsigned attributes);
    virtual ~VGMesh() {}

    VGMesh(const Self& other);

    VGMesh& operator=(const Self& rhs);
    VGMesh& assign(const Self& rhs);

    /// \}


    /// \name Mesh and general
    /// \{

    inline unsigned nDims() const { return m_positions.rows(); }
    inline unsigned nCoeffs() const { return m_nodes.rows(); }

    void setNDims(unsigned nDims);

protected:
    inline unsigned positionsCapacity() const { return m_positions.cols(); }

public:

    /**
     * \brief Set the number of coefficients to nCoeffs.
     *
     * If this mesh has a fixed number of coefficients, nCoeffs must match it.
     *
     * If this function is used to increase the number of coefficients of a
     * non-empty mesh, be aware that the new coefficients will be
     * uninitialized. In other words, value() will return partially initialized
     * vectors.
     */
    void setNCoeffs(unsigned nCoeffs);

    inline void reserve(unsigned nvertices, unsigned nedges, unsigned nfaces,
                        unsigned nnodes);
    inline void clear();
    void garbageCollection(unsigned flags = 0);
    void releaseGCIndexMaps();

    using PatateCommon::SurfaceMesh::gcMap;
    inline Node gcMap(Node n) const
    {
        assert(n.isValid() && n.idx() < int(m_gcNodeMap.size()));
        return m_gcNodeMap[n.idx()];
    }

    template <typename Derived>
    inline Vertex addVertex(const Eigen::DenseBase<Derived>& pos);

    inline ConstVectorXpr position(Vertex v) const
    { assert(v.idx() < verticesSize()); return m_positions.col(v.idx()); }

    inline VectorXpr position(Vertex v)
    { assert(v.idx() < verticesSize()); return m_positions.col(v.idx()); }

//    inline VertexProperty<Vector>& positionProperty() { return m_positions; }
//    inline const VertexProperty<Vector>& positionProperty() const
//        { return m_positions; }

//    inline bool isValid(Vertex v) const { return PatateCommon::SurfaceMesh::isValid(v); }
//    inline bool isValid(Halfedge h) const { return PatateCommon::SurfaceMesh::isValid(h); }
//    inline bool isValid(Edge e) const { return PatateCommon::SurfaceMesh::isValid(e); }
//    inline bool isValid(Face f) const { return PatateCommon::SurfaceMesh::isValid(f); }

    using PatateCommon::SurfaceMesh::isValid;
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

    template <class T> NodeProperty<T> addNodeProperty(const std::string& name, const T t=T())
    {
        return NodeProperty<T>(m_nprops.add<T>(name, t));
    }

    template <class T> NodeProperty<T> getNodeProperty(const std::string& name) const
    {
        return NodeProperty<T>(m_nprops.get<T>(name));
    }

    template <class T> NodeProperty<T> nodeProperty(const std::string& name, const T t=T())
    {
        return NodeProperty<T>(m_nprops.getOrAdd<T>(name, t));
    }

    template <class T> void removeNodeProperty(NodeProperty<T>& p)
    {
        m_nprops.remove(p);
    }

    const std::type_info& getNodePropertyType(const std::string& name)
    {
        return m_nprops.getType(name);
    }

    std::vector<std::string> nodeProperties() const
    {
        return m_nprops.properties();
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

    inline bool hasAttribute(HalfedgeAttribute attr) const
        { assert(attr < HALFEDGE_ATTRIB_COUNT); return m_attributes & (1 << attr); }
    inline bool hasToVertexValue() const { return m_attributes & TO_VERTEX_VALUE_FLAG; }
    inline bool hasFromVertexValue() const { return m_attributes & FROM_VERTEX_VALUE_FLAG; }
    inline bool hasEdgeValue() const { return m_attributes & EDGE_VALUE_FLAG; }
    inline bool hasEdgeGradient() const { return m_attributes & EDGE_GRADIENT_FLAG; }
    inline bool hasVertexGradientConstraint() const
        { return m_attributes & VERTEX_GRADIENT_CONSTRAINT_FLAG; }


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


    /// \name Point gradient constraints
    /// \{

    inline bool isGradientConstraint(Vertex v) const
        { assert(hasVertexGradientConstraint()); return m_vertexGradientConstraints.count(v); }
    inline Gradient& gradientConstraint(Vertex v)
        { assert(hasVertexGradientConstraint()); return m_vertexGradientConstraints.at(v); }
    inline const Gradient& gradientConstraint(Vertex v) const
        { assert(hasVertexGradientConstraint()); return m_vertexGradientConstraints.at(v); }
    inline void setGradientConstraint(Vertex v, const Gradient& grad);
    inline void removeGradientConstraint(Vertex v);

    inline int nVertexGradientConstraints() const
        { assert(hasVertexGradientConstraint()); return m_vertexGradientConstraints.size(); }
    inline unsigned nVertexGradientConstraints(Halfedge h) const;
    inline unsigned nVertexGradientConstraints(Face f) const;

    /// \}


    /// \name Low-level nodes manipulation
    /// \{

    inline unsigned nodesSize() const { return m_nprops.size(); }
    /// \brief Return the number of nodes.
    inline unsigned nNodes() const { return nodesSize() - m_deletedNodes; }

protected:
    inline unsigned nodesCapacity() const { return m_nodes.cols(); }

public:
    using PatateCommon::SurfaceMesh::isDeleted;
    inline bool isDeleted(Node n) const { return m_ndeleted[n]; }

    /// \brief Mark unused node as deleted, but does not free memory.
    /// \sa garbageCollection()
    void deleteUnusedNodes();

    NodeIterator nodesBegin() const { return NodeIterator(Node(0),           this); }
    NodeIterator nodesEnd()   const { return NodeIterator(Node(nodesSize()), this); }

    /**
     * \brief Add and return a node with value `value` or an unknown node
     * if no parameter is provided.
     */
    inline Node addNode() { return addNode(unconstrainedValue()); }
    template <typename Derived>
    inline Node addNode(const Eigen::DenseBase<Derived>& value);

    /// \brief Read only access to the value of `node`.
    inline ConstValueXpr value(Node node) const
    { assert(node.idx() < nodesSize()); return m_nodes.col(node.idx()); }

    /// \brief Read-write access to the value of `node`.
    inline ValueXpr value(Node node)
    { assert(node.idx() < nodesSize()); return m_nodes.col(node.idx()); }

    inline UnconstrainedNodeType unconstrainedValue() const
    { return Value::Constant(nCoeffs(), std::numeric_limits<Scalar>::quiet_NaN()); }

    /// \brief Return true iff `node` is a constraint.
    inline bool isConstraint(Node node) const
    { return isValid(node) && !std::isnan(value(node)[0]); }

    /**
     * \brief Return true if the mesh has unknown nodes.
     * \par Complexity
     * This function operates in \f$O(n)\f$ with \f$n\f$ the number of nodes.
     */
    inline bool hasUnknowns() const;
    inline unsigned nUnknowns() const;

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
     * One typically wish to call VGMesh::deleteUnusedNodes() and
     * VGMesh::garbageCollection after this function to effectively remove
     * unused nodes.
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
    void simplifyOppositeNodes(Node& n0, Node& n1,
                               bool b0 = false, bool b1 = false) const;

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
    // FIXME: Would likely be better to use a hash map, but this is only
    // available in C++11...
    typedef std::map<Vertex, Gradient> VertexGradientMap;
    typedef std::map<Halfedge, Node> HalfedgeNodeMap;

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

    void copyVGMeshMembers(const Self& rhs);
    void findConstrainedEdgesSimplify(Vertex vx,
                                      std::vector<Halfedge>& consEdges);
    void resizePositionsMatrix(unsigned rows, unsigned cols);
    void resizeNodesMatrix(unsigned rows, unsigned cols);

    /// \}

protected:
    unsigned m_attributes;

    PatateCommon::PropertyContainer m_nprops;

    VectorMatrix m_positions;
    VertexGradientMap m_vertexGradientConstraints;

    HalfedgeProperty<Node> m_toVertexValueNodes;
    HalfedgeProperty<Node> m_fromVertexValueNodes;
    HalfedgeProperty<Node> m_edgeValueNodes;
    HalfedgeProperty<Node> m_edgeGradientNodes;

    unsigned m_deletedNodes;
    NodeMatrix m_nodes;
    NodeProperty<bool> m_ndeleted;
    std::vector<Node> m_gcNodeMap;
};

} // namespace Vitelotte

#include "vgMesh.hpp"


#endif // VGMESH_H

