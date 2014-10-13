
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
QuadraticMesh<_Scalar, _Dim, _Chan>::sortAndCompactNodes()
{
    std::vector<NodeID> buf(m_nodes.size(), 0);

    // Find used node ids
    HalfedgeIterator hBegin = halfedgesBegin(),
                     hEnd   = halfedgesEnd();
    for(HalfedgeIterator hIt = hBegin; hIt != hEnd; ++hIt)
    {
        if(isBoundary(*hIt))
            continue;
        buf[fromNode(*hIt)]     = 1;
        buf[toNode(*hIt)]       = 1;
        buf[midNode(*hIt)]      = 1;
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

    // Sort remaining nodes
    buf.resize(size);
    NodeCompare cmp(*this);
    std::sort(buf.begin(), buf.end(), cmp);

    // Update node vector and fill remapping vector
    std::vector<NodeID> map(m_nodes.size(), InvalidNodeID);
    NodeVector reord(size);
    for(NodeID i = 0; i < size; ++i)
    {
        reord[i] = nodeValue(buf[i]);
        map[buf[i]] = i;
    }
    m_nodes.swap(reord);

    // Remap nodes in mesh
    for(HalfedgeIterator hIt = hBegin; hIt != hEnd; ++hIt)
    {
        if(isBoundary(*hIt))
            continue;
        fromNode(*hIt)     = map[fromNode(*hIt)];
        toNode(*hIt)       = map[toNode(*hIt)];
        midNode(*hIt)      = map[midNode(*hIt)];
    }
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
