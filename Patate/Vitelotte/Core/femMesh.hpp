#include "femMesh.h"


namespace Vitelotte
{


template < typename _Scalar, int _Dim, int _Chan >
const typename FemMesh<_Scalar, _Dim, _Chan>::NodeValue
    FemMesh<_Scalar, _Dim, _Chan>::UnconstrainedNode =
        FemMesh<_Scalar, _Dim, _Chan>::NodeValue::Constant(
            std::numeric_limits<Scalar>::quiet_NaN());


template < typename _Scalar, int _Dim, int _Chan >
FemMesh<_Scalar, _Dim, _Chan>::FemMesh(unsigned attributes)
    : m_attributes(0)
{
    m_positions = addVertexProperty<Vector>("v:position", Vector::Zero());

    setAttributes(attributes);
}


template < typename _Scalar, int _Dim, int _Chan >
template < typename OtherScalar >
FemMesh<_Scalar, _Dim, _Chan>&
FemMesh<_Scalar, _Dim, _Chan>::operator=(
        const FemMesh<OtherScalar, _Dim, _Chan>& rhs)
{
    if(&rhs != this)
    {
        // FIXME: SurfaceMesh's operator= wont work with properties of different types.
        Patate::SurfaceMesh::operator=(rhs);

        m_nodes = rhs.m_nodes;

        // TODO: Working copy constructor
//        m_vPos      = vertexProperty<Vector>("v:position");

//        m_hFromNode = halfedgeProperty<Node>("h:fromNode");
//        m_hToNode   = halfedgeProperty<Node>("h:toNode");
//        m_hMidNode  = halfedgeProperty<Node>("h:midNode");
    }
    return *this;
}


template < typename _Scalar, int _Dim, int _Chan >
void FemMesh<_Scalar, _Dim, _Chan>::setAttributes(unsigned attributes)
{
#define PATATE_FEM_MESH_SET_ATTRIBUTE(_bit, _field, _name) do {\
        if(!(m_attributes & _bit) && (attributes & _bit))\
            _field = addHalfedgeProperty<Node>(_name, Node());\
        else if((m_attributes & _bit) && !(attributes & _bit))\
            removeHalfedgeProperty(_field);\
    } while(0)

    PATATE_FEM_MESH_SET_ATTRIBUTE(
                VertexValue,     m_vertexValueNodes,     "h:vertexValueNode");
    PATATE_FEM_MESH_SET_ATTRIBUTE(
                VertexFromValue, m_vertexFromValueNodes, "h:vertexFromValueNode");
    PATATE_FEM_MESH_SET_ATTRIBUTE(
                EdgeValue,       m_edgeValueNodes,       "h:edgeValueNode");
    PATATE_FEM_MESH_SET_ATTRIBUTE(
                EdgeGradient,    m_edgeGradientNodes,    "h:edgeGradientNode");

//    PATATE_FEM_MESH_SET_ATTRIBUTE(
//                VertexGradientSpecial, m_vertexGradientSpecial, "h:vertexGradientSpecial");

    m_attributes = attributes;

#undef PATATE_FEM_MESH_SET_ATTRIBUTE
}


template < typename _Scalar, int _Dim, int _Chan >
void
FemMesh<_Scalar, _Dim, _Chan>::setEdgeConstraintFlag(bool on)
{
    if(on && !hasEdgeConstraintFlag())
        m_edgeConstraintFlag = edgeProperty("e:constraintFlag", false);
    else if(!on && hasEdgeConstraintFlag())
        removeEdgeProperty(m_edgeConstraintFlag);
}


template < typename _Scalar, int _Dim, int _Chan >
typename FemMesh<_Scalar, _Dim, _Chan>::Node
FemMesh<_Scalar, _Dim, _Chan>::addNode(const NodeValue& nodeValue)
{
    m_nodes.push_back(nodeValue);
    return Node(m_nodes.size() - 1);
}


template < typename _Scalar, int _Dim, int _Chan >
void
FemMesh<_Scalar, _Dim, _Chan>::compactNodes()
{
    std::vector<Node> buf(nNodes(), 0);

    // Find used node ids
    HalfedgeIterator hBegin = halfedgesBegin(),
                     hEnd   = halfedgesEnd();
    for(HalfedgeIterator hit = hBegin; hit != hEnd; ++hit)
    {
        if(!isBoundary(*hit))
        {
            if(hasVertexValue())
                buf[vertexValueNode(*hit)]     = 1;
            if(hasVertexFromValue())
                buf[vertexFromValueNode(*hit)] = 1;
            if(hasEdgeValue())
                buf[edgeValueNode(*hit)]      = 1;
//            if(hasVertexGradient())
//                marked[vertexGradientNode(*hit)]  = 1;
            if(hasEdgeGradient())
                buf[edgeGradientNode(*hit)]  = 1;
        }
    }

    // Compute remapping
    Node size=0;
    for(Node i = 0; i < buf.size(); ++i)
    {
        if(buf[i])
        {
            buf[size] = i;
            ++size;
        }
    }

    // Update node vector and fill remapping vector
    std::vector<Node> map(nNodes(), Node());
    NodeVector reord(size);
    for(Node i = 0; i < size; ++i)
    {
        reord[i] = nodeValue(buf[i]);
        map[buf[i]] = i;
    }
    m_nodes.swap(reord);

    // Remap nodes in mesh
    for(HalfedgeIterator hit = hBegin; hit != hEnd; ++hit)
    {
        if(!isBoundary(*hit))
        {
            if(hasVertexValue())
                vertexValueNode(*hit)     = map[vertexValueNode(*hit)];
            if(hasVertexFromValue())
                vertexFromValueNode(*hit) = map[vertexFromValueNode(*hit)];
            if(hasEdgeValue())
                (edgeValueNode(*hit))      = map[edgeValueNode(*hit)];
//            if(hasVertexGradient())
//                vertexGradientNode(*hit)  = map[vertexGradientNode(*hit)];
            if(hasEdgeGradient())
                edgeGradientNode(*hit)  = map[edgeGradientNode(*hit)];
        }
    }
}


//template < typename _Scalar, int _Dim, int _Chan >
//void
//FemMesh<_Scalar, _Dim, _Chan>::setValueConstraint(
//        Halfedge h, Node from, Node mid, Node to)
//{
//    vertexValueNode(h) = to;
//    edgeGradientNode(h) = mid;
//    vertexFromValueNode(h) = from;
//}


//template < typename _Scalar, int _Dim, int _Chan >
//bool
//FemMesh<_Scalar, _Dim, _Chan>::isValueConstraint(Edge e) const
//{
//    Halfedge h0 = halfedge(e, 0);
//    Halfedge h1 = halfedge(e, 1);
//    bool b0 = isBoundary(h0);
//    bool b1 = isBoundary(h1);
//    return
//        (!b0 && !b1 &&
//            (fromNode(h0) != toNode(h1) ||
//             midNode(h0) != midNode(h1) ||
//             toNode(h0) != fromNode(h1))) ||
//        (!b0 &&
//            (isConstraint(fromNode(h0)) ||
//             isConstraint( midNode(h0)) ||
//             isConstraint(  toNode(h0)))) ||
//        (!b1 &&
//            (isConstraint(fromNode(h1)) ||
//             isConstraint( midNode(h1)) ||
//             isConstraint(  toNode(h1))));
//}


template < typename _Scalar, int _Dim, int _Chan >
void
FemMesh<_Scalar, _Dim, _Chan>::
    setVertexNode(Node node, Halfedge from, Halfedge to)
{
    assert(fromVertex(from) == fromVertex(to));
    assert(hasVertexValue());

    Halfedge h = from;
    do
    {
        if(hasVertexFromValue() && !isBoundary(h))
            vertexFromValueNode(h) = node;

        h = prevHalfedge(h);

        if(!isBoundary(h))
            vertexValueNode(h) = node;

        h = oppositeHalfedge(h);
    }
    while(h != to && h != from);
}


template < typename _Scalar, int _Dim, int _Chan >
void
FemMesh<_Scalar, _Dim, _Chan>::
    setSingularity(Node fromNode, Node toNode, Halfedge from, Halfedge to)
{
    assert(fromVertex(from) == fromVertex(to));
    assert(hasVertexValue() && hasVertexFromValue());
    assert(isConstraint(fromNode) && isConstraint(toNode));

    const NodeValue& fromValue = nodeValue(fromNode);
    const NodeValue&   toValue = nodeValue(  toNode);

    const Vector& v = position(fromVertex(from));
    Vector fromVec = position(toVertex(from)) - v;
    Vector   toVec = position(toVertex(  to)) - v;

    Scalar fromAngle = std::atan2(fromVec.y(), fromVec.x());
    Scalar toAngle   = std::atan2(  toVec.y(),   toVec.x());
    Scalar totalAngle = toAngle - fromAngle;
    if(totalAngle < 1.e-8) totalAngle += 2.*M_PI;

    Node n = fromNode;
    Halfedge h = from;
    Halfedge last = oppositeHalfedge(to);
    do
    {
        if(!isBoundary(h))
            vertexFromValueNode(h) = n;

        h = prevHalfedge(h);

        if(h != last)
        {
            // Current Halfedge is reversed.
            Vector vec = position(fromVertex(h)) - v;
            Scalar angle = std::atan2(vec.y(), vec.x()) - fromAngle;
            if(angle < 1.e-8) angle += 2.*M_PI;
            Scalar a = angle / totalAngle;
            n = addNode((1.-a) * fromValue + a * toValue);
        }
        else
            n = toNode;

        if(!isBoundary(h))
            vertexValueNode(h) = n;

        h = oppositeHalfedge(h);
    }
    while(h != to && h != from);
}


template < typename _Scalar, int _Dim, int _Chan >
void
FemMesh<_Scalar, _Dim, _Chan>::finalize()
{
    assert(hasVertexValue());

    std::vector<Halfedge> consEdges;
    consEdges.reserve(8);

    for(VertexIterator vit = verticesBegin();
        vit != verticesEnd(); ++vit)
    {
        consEdges.clear();
        HalfedgeAroundVertexCirculator hit = halfedges(*vit),
                                       hEnd = hit;
        do
        {
            if(vertexValueNode(oppositeHalfedge(*hit)).isValid() ||
                    (hasVertexFromValue() && vertexFromValueNode(*hit).isValid()))
                consEdges.push_back(*hit);
            ++hit;
        }
        while(hit != hEnd);

        if(consEdges.empty())
        {
            consEdges.push_back(*hit);
            consEdges.push_back(*hit);
        }
        consEdges.push_back(consEdges.front());

        while(*hit != consEdges.front()) ++hit;

        std::vector<Halfedge>::iterator cit = consEdges.begin();
        Halfedge prev = *cit;
        for(++cit; cit != consEdges.end(); ++cit)
        {
            assert(prev == *hit);
            Halfedge next = oppositeHalfedge(*cit);
            Node n0 = hasVertexFromValue()?
                        vertexFromValueNode(prev):
                        vertexValueNode(prevHalfedge(prev));
            Node n1 = vertexValueNode(next);
            bool n0c = n0.isValid() && isConstraint(n0);
            bool n1c = n1.isValid() && isConstraint(n1);

            Node n = Node();
            if(!n0c && !n1c)
            {
                // Free nodes, choose one valid or create a new one.
                if     (n0.isValid()) n = n0;
                else if(n1.isValid()) n = n1;
                else                  n = addNode();
            }
            else if(n0c != n1c)
                n = n0c? n0: n1;  // One constraint, choose it
            else if(n0 == n1)
                n = n0;  // Same constraints, choice is obvious

            // The remaining option is a singularity, that require special
            // processing.
            if(isValid(n))
                setVertexNode(n, prev, *cit);
            else
                setSingularity(n0, n1, prev, *cit);

            prev = *cit;
        }
    }

    if(hasEdgeValue() || hasEdgeGradient())
    {
        for(EdgeIterator eit = edgesBegin();
            eit != edgesEnd(); ++eit)
        {
            if(hasEdgeValue())
            {
                Node& n0 = edgeValueNode(halfedge(*eit, false));
                Node& n1 = edgeValueNode(halfedge(*eit, true));
                bool n0v = n0.isValid();
                bool n1v = n1.isValid();

                // if both are invalid, create a single node. Else, create
                // nodes independently, thus producing a discontinuity.
                if(!n0v) n0 = addNode();
                if(!n1v) n1 = n0v? addNode(): n0;
            }
            if(hasEdgeGradient())
            {
                Node& n0 = edgeGradientNode(halfedge(*eit, false));
                Node& n1 = edgeGradientNode(halfedge(*eit, true));
                bool n0v = n0.isValid();
                bool n1v = n1.isValid();

                // if both are invalid, create a single node. Else, create
                // nodes independently, thus producing a discontinuity.
                if(!n0v) n0 = addNode();
                if(!n1v) n1 = n0v? addNode(): n0;
            }
        }
    }
}


template < typename _Scalar, int _Dim, int _Chan >
bool
FemMesh<_Scalar, _Dim, _Chan>::isSingular(Halfedge h) const
{
    return hasVertexFromValue() &&
            vertexValueNode(h) != vertexFromValueNode(nextHalfedge(h));
}


template < typename _Scalar, int _Dim, int _Chan >
bool
FemMesh<_Scalar, _Dim, _Chan>::isSingular(Face f) const
{
    HalfedgeAroundFaceCirculator
            hit  = halfedges(f),
            hend = hit;
    do
    {
        if(isSingular(*hit))
            return true;
    }
    while(++hit != hend);

    return false;
}


template < typename _Scalar, int _Dim, int _Chan >
unsigned
FemMesh<_Scalar, _Dim, _Chan>::nSingularFaces() const
{
    unsigned nSingulars = 0;
    for(FaceIterator fit = facesBegin();
        fit != facesEnd(); ++fit)
    {
        nSingulars += isSingular(*fit);
    }
    return nSingulars;
}


template < typename _Scalar, int _Dim, int _Chan >
void
FemMesh<_Scalar, _Dim, _Chan>::reserve(
        unsigned nvertices, unsigned nedges, unsigned nfaces, unsigned nnodes)
{
    Patate::SurfaceMesh::reserve(nvertices, nedges, nfaces);
    m_nodes.reserve(nnodes);
}


template < typename _Scalar, int _Dim, int _Chan >
void
FemMesh<_Scalar, _Dim, _Chan>::clear()
{
    Patate::SurfaceMesh::clear();
    m_nodes.clear();
}


template < typename _Scalar, int _Dim, int _Chan >
Patate::SurfaceMesh::Vertex
FemMesh<_Scalar, _Dim, _Chan>::addVertex(const Vector& pos)
{
    Vertex v = Patate::SurfaceMesh::addVertex();
    position(v) = pos;
    return v;
}


template < typename _Scalar, int _Dim, int _Chan >
bool
FemMesh<_Scalar, _Dim, _Chan>::isValid(Node n) const
{
    return 0 <= n.idx() && n.idx() < m_nodes.size();
}


}  // namespace Vitelotte
