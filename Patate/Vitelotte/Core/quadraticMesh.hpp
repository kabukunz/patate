
namespace internal
{

template < typename Mesh >
void
compactNodes(Mesh& mesh)
{
    typedef typename Mesh::NodeID NodeID;
    typedef typename Mesh::HalfedgeIterator HalfedgeIterator;
    typedef typename Mesh::NodeVector NodeVector;

    std::vector<NodeID> buf(mesh.nNodes(), 0);

    // Find used node ids
    HalfedgeIterator hBegin = mesh.halfedgesBegin(),
                     hEnd   = mesh.halfedgesEnd();
    for(HalfedgeIterator hit = hBegin; hit != hEnd; ++hit)
    {
        if(!mesh.isBoundary(*hit))
            mesh.markNodes(*hit, buf);
    }

    // Compute remapping
    NodeID size=0;
    for(NodeID i = 0; i < buf.size(); ++i)
    {
        if(buf[i])
        {
            buf[size] = i;
            ++size;
        }
    }

    // Update node vector and fill remapping vector
    std::vector<NodeID> map(mesh.nNodes(), Mesh::InvalidNodeID);
    NodeVector reord(size);
    for(NodeID i = 0; i < size; ++i)
    {
        reord[i] = mesh.nodeValue(buf[i]);
        map[buf[i]] = i;
    }
    mesh.m_nodes.swap(reord);

    // Remap nodes in mesh
    for(HalfedgeIterator hit = hBegin; hit != hEnd; ++hit)
    {
        if(!mesh.isBoundary(*hit))
            mesh.remapNodes(*hit, map);
    }
}

} // namespace internal


template < typename _Scalar, int _Dim, int _Chan >
const typename QuadraticMesh<_Scalar, _Dim, _Chan>::NodeValue
    QuadraticMesh<_Scalar, _Dim, _Chan>::UnconstrainedNode =
        QuadraticMesh<_Scalar, _Dim, _Chan>::NodeValue::Constant(
            std::numeric_limits<Scalar>::quiet_NaN());

template < typename _Scalar, int _Dim, int _Chan >
QuadraticMesh<_Scalar, _Dim, _Chan>::QuadraticMesh()
{
    m_vPos      = addVertexProperty<Vector>("v:position", Vector::Zero());

    m_hFromNode = addHalfedgeProperty<NodeID>("h:fromNode", InvalidNodeID);
    m_hToNode   = addHalfedgeProperty<NodeID>("h:toNode", InvalidNodeID);
    m_hMidNode  = addHalfedgeProperty<NodeID>("h:midNode", InvalidNodeID);
}

template < typename _Scalar, int _Dim, int _Chan >
template < typename OtherScalar >
QuadraticMesh<_Scalar, _Dim, _Chan>&
QuadraticMesh<_Scalar, _Dim, _Chan>::operator=(
        const QuadraticMesh<OtherScalar, _Dim, _Chan>& rhs)
{
    if(&rhs != this)
    {
        // FIXME: SurfaceMesh's operator= wont work with properties of different types.
        Patate::SurfaceMesh::operator=(rhs);

        m_nodes = rhs.m_nodes;

        m_vPos      = vertexProperty<Vector>("v:position");

        m_hFromNode = halfedgeProperty<NodeID>("h:fromNode");
        m_hToNode   = halfedgeProperty<NodeID>("h:toNode");
        m_hMidNode  = halfedgeProperty<NodeID>("h:midNode");
    }
    return *this;
}

template < typename _Scalar, int _Dim, int _Chan >
typename QuadraticMesh<_Scalar, _Dim, _Chan>::NodeID
QuadraticMesh<_Scalar, _Dim, _Chan>::addNode(const NodeValue& nodeValue)
{
    m_nodes.push_back(nodeValue);
    return m_nodes.size() - 1;
}

template < typename _Scalar, int _Dim, int _Chan >
void
QuadraticMesh<_Scalar, _Dim, _Chan>::compactNodes()
{
    internal::compactNodes(*this);
}

template < typename _Scalar, int _Dim, int _Chan >
template < typename Marked >
void
QuadraticMesh<_Scalar, _Dim, _Chan>::markNodes(Halfedge h, Marked& marked) const
{
    marked[fromNode(h)]     = 1;
    marked[toNode(h)]       = 1;
    marked[midNode(h)]      = 1;
}

template < typename _Scalar, int _Dim, int _Chan >
template < typename Map >
void
QuadraticMesh<_Scalar, _Dim, _Chan>::remapNodes(Halfedge h, Map& map)
{
    fromNode(h)     = map[fromNode(h)];
    toNode(h)       = map[toNode(h)];
    midNode(h)      = map[midNode(h)];
}

template < typename _Scalar, int _Dim, int _Chan >
void
QuadraticMesh<_Scalar, _Dim, _Chan>::initializeValueConstraints()
{
    m_nodes.clear();

    for(VertexIterator vit = verticesBegin();
        vit != verticesEnd(); ++vit)
    {
        NodeID node = addNode();
        HalfedgeAroundVertexCirculator hit = halfedges(*vit),
                                       hEnd = hit;
        do
        {
            if(isValid(*hit))
                fromNode(*hit) = node;
            if(isValid(oppositeHalfedge(*hit)))
                toNode(oppositeHalfedge(*hit)) = node;
            ++hit;
        }
        while(hit != hEnd && !isBoundary(*hit));
    }

    for(EdgeIterator eit = edgesBegin();
        eit != edgesEnd(); ++eit)
    {
        NodeID node = addNode();
        Halfedge h = halfedge(*eit, 0);
        if(isValid(h))
            midNode(h) = node;
        if(isValid(oppositeHalfedge(h)))
            midNode(oppositeHalfedge(h)) = node;
    }
}

template < typename _Scalar, int _Dim, int _Chan >
void
QuadraticMesh<_Scalar, _Dim, _Chan>::setValueConstraint(
        Halfedge h, NodeID from, NodeID mid, NodeID to)
{
    fromNode(h) = from;
    midNode(h) = mid;
    toNode(h) = to;
}

template < typename _Scalar, int _Dim, int _Chan >
bool
QuadraticMesh<_Scalar, _Dim, _Chan>::isValueConstraint(Edge e) const
{
    Halfedge h0 = halfedge(e, 0);
    Halfedge h1 = halfedge(e, 1);
    bool b0 = isBoundary(h0);
    bool b1 = isBoundary(h1);
    return
        (!b0 && !b1 &&
            (fromNode(h0) != toNode(h1) ||
             midNode(h0) != midNode(h1) ||
             toNode(h0) != fromNode(h1))) ||
        (!b0 &&
            (isConstraint(fromNode(h0)) ||
             isConstraint( midNode(h0)) ||
             isConstraint(  toNode(h0)))) ||
        (!b1 &&
            (isConstraint(fromNode(h1)) ||
             isConstraint( midNode(h1)) ||
             isConstraint(  toNode(h1))));
}

template < typename _Scalar, int _Dim, int _Chan >
void
QuadraticMesh<_Scalar, _Dim, _Chan>::propagateConstraints()
{
    std::vector<Halfedge> consEdges;
    consEdges.reserve(8);

    // TODO: Destroy this property ?
    EdgeProperty<bool> constraintMap = edgeProperty("e:isConstraint", false);
    for(EdgeIterator eit = edgesBegin();
        eit != edgesEnd(); ++eit)\
    {
        constraintMap[*eit] = isValueConstraint(*eit);
    }

    for(VertexIterator vit = verticesBegin();
        vit != verticesEnd(); ++vit)
    {
        consEdges.clear();
        HalfedgeAroundVertexCirculator hit = halfedges(*vit),
                                       hEnd = hit;
        do
        {
            if(constraintMap[edge(*hit)])
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
            NodeID n0 = fromNode(prev);
            NodeID n1 = toNode(next);
            bool n0c = isConstraint(n0);
            bool n1c = isConstraint(n1);

            NodeID n = InvalidNodeID;
            if(!n0c && !n1c)
                n = n0;  // Free nodes, choose one arbitrarily
            else if(n0c != n1c)
                n = n0c? n0: n1;  // One constraint, choose it
            else if(n0 == n1)
                n = n0;  // Same constraints, choice is obvious

            // The remaining option is a singularity, that require special
            // processing.
            if(isValid(n))
            {
                do
                {
                    if(isValid(*hit))
                        fromNode(*hit) = n;
                    ++hit;
                    if(isValid(oppositeHalfedge(*hit)))
                    toNode(oppositeHalfedge(*hit)) = n;
                }
                while(*hit != *cit);
            }
            else
                processSingularity(hit, *cit);

            prev = *cit;
        }
    }
}

template < typename _Scalar, int _Dim, int _Chan >
void
QuadraticMesh<_Scalar, _Dim, _Chan>::
    processSingularity(HalfedgeAroundVertexCirculator& hit, Halfedge end)
{
    NodeValue from = nodeValue(fromNode(*hit));
    NodeValue to = nodeValue(toNode(oppositeHalfedge(end)));

    Vector fromVec = position(toVertex(*hit)) - position(fromVertex(*hit));
    Vector   toVec = position(toVertex( end)) - position(fromVertex( end));

    Scalar fromAngle = std::atan2(fromVec.y(), fromVec.x());
    Scalar toAngle   = std::atan2(  toVec.y(),   toVec.x());
    Scalar totalAngle = toAngle - fromAngle;
    if(totalAngle < 1.e-8) totalAngle += 2.*M_PI;

    NodeID n = fromNode(*hit);
    do
    {
        if(isValid(*hit))
            fromNode(*hit) = n;
        ++hit;

        if(*hit != end)
        {
            Vector vec = position(toVertex(*hit)) - position(fromVertex(*hit));
            Scalar angle = std::atan2(vec.y(), vec.x()) - fromAngle;
            if(angle < 1.e-8) angle += 2.*M_PI;
            Scalar a = angle / totalAngle;
            n = addNode((1.-a) * from + a * to);
            if(isValid(oppositeHalfedge(*hit)))
                toNode(oppositeHalfedge(*hit)) = n;
        }
    }
    while(*hit != end);
}

template < typename _Scalar, int _Dim, int _Chan >
bool
QuadraticMesh<_Scalar, _Dim, _Chan>::isSingular(Halfedge h) const
{
    return m_hToNode[h] != m_hFromNode[nextHalfedge(h)];
}

template < typename _Scalar, int _Dim, int _Chan >
bool
QuadraticMesh<_Scalar, _Dim, _Chan>::isSingular(Face f) const
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
QuadraticMesh<_Scalar, _Dim, _Chan>::nSingularFaces() const
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
QuadraticMesh<_Scalar, _Dim, _Chan>::reserve(
        unsigned nvertices, unsigned nedges, unsigned nfaces, unsigned nnodes)
{
    Patate::SurfaceMesh::reserve(nvertices, nedges, nfaces);
    m_nodes.reserve(nnodes);
}

template < typename _Scalar, int _Dim, int _Chan >
void
QuadraticMesh<_Scalar, _Dim, _Chan>::clear()
{
    Patate::SurfaceMesh::clear();
    m_nodes.clear();
}

template < typename _Scalar, int _Dim, int _Chan >
Patate::SurfaceMesh::Vertex
QuadraticMesh<_Scalar, _Dim, _Chan>::addVertex(const Vector& pos)
{
    Vertex v = Patate::SurfaceMesh::addVertex();
    m_vPos[v] = pos;
    return v;
}

template < typename _Scalar, int _Dim, int _Chan >
bool
QuadraticMesh<_Scalar, _Dim, _Chan>::isValid(NodeID n) const
{
    return 0 <= n && n < m_nodes.size();
}
