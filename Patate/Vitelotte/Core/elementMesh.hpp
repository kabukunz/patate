
template < typename _Scalar, int _Dim, int _Chan >
const typename ElementMesh<_Scalar, _Dim, _Chan>::NodeValue
    ElementMesh<_Scalar, _Dim, _Chan>::UnconstrainedNode =
        ElementMesh<_Scalar, _Dim, _Chan>::NodeValue::Constant(
            std::numeric_limits<Scalar>::quiet_NaN());

template < typename _Scalar, int _Dim, int _Chan >
ElementMesh<_Scalar, _Dim, _Chan>::ElementMesh()
{
    m_vPos      = addVertexProperty<Vector>("v:position", Vector::Zero());
    m_vFlatGrad = addVertexProperty<bool>("v:flatGrad", false);

    m_hFromNode = addHalfedgeProperty<NodeID>("h:fromNode", InvalidNodeID);
    m_hToNode   = addHalfedgeProperty<NodeID>("h:toNode", InvalidNodeID);
    m_hMidNode  = addHalfedgeProperty<NodeID>("h:midNode", InvalidNodeID);
    m_hGradNode = addHalfedgeProperty<NodeID>("h:gradNode", InvalidNodeID);
}

template < typename _Scalar, int _Dim, int _Chan >
template < typename OtherScalar >
ElementMesh<_Scalar, _Dim, _Chan>&
ElementMesh<_Scalar, _Dim, _Chan>::operator=(
        const ElementMesh<OtherScalar, _Dim, _Chan>& rhs)
{
    if(&rhs != this)
    {
        // FIXME: SurfaceMesh's operator= wont work with properties of different types.
        Patate::SurfaceMesh::operator=(rhs);

        m_nodes = rhs.m_nodes;

        m_vPos      = vertexProperty<Vector>("v:position");
        m_vFlatGrad = vertexProperty<bool>("v:flatGrad");

        m_hFromNode = halfedgeProperty<NodeID>("h:fromNode");
        m_hToNode   = halfedgeProperty<NodeID>("h:toNode");
        m_hMidNode  = halfedgeProperty<NodeID>("h:midNode");
        m_hGradNode = halfedgeProperty<NodeID>("h:gradNode");
    }
    return *this;
}

template < typename _Scalar, int _Dim, int _Chan >
typename ElementMesh<_Scalar, _Dim, _Chan>::NodeID
ElementMesh<_Scalar, _Dim, _Chan>::addNode(const NodeValue& nodeValue)
{
    m_nodes.push_back(nodeValue);
    return m_nodes.size() - 1;
}

template < typename _Scalar, int _Dim, int _Chan >
void
ElementMesh<_Scalar, _Dim, _Chan>::sortAndCompactNodes()
{
    std::vector<NodeID> buf(m_nodes.size(), 0);

    // Find used node ids
    FaceIterator fBegin = facesBegin(),
                 fEnd   = facesEnd();
    for(FaceIterator fIt = fBegin; fIt != fEnd; ++fIt)
    {
        HalfedgeAroundFaceCirculator hBegin = halfedges(*fIt),
                                     hIt    = hBegin;
        do
        {
            buf[fromNode(*hIt)]     = 1;
            buf[toNode(*hIt)]       = 1;
            buf[midNode(*hIt)]      = 1;
            buf[gradientNode(*hIt)] = 1;
            ++hIt;
        } while(hIt != hBegin);
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
    for(FaceIterator fIt = fBegin; fIt != fEnd; ++fIt)
    {
        HalfedgeAroundFaceCirculator hBegin = halfedges(*fIt),
                                     hIt    = hBegin;
        do
        {
            fromNode(*hIt)     = map[fromNode(*hIt)];
            toNode(*hIt)       = map[toNode(*hIt)];
            midNode(*hIt)      = map[midNode(*hIt)];
            gradientNode(*hIt) = map[gradientNode(*hIt)];
            ++hIt;
        } while(hIt != hBegin);
    }
}

template < typename _Scalar, int _Dim, int _Chan >
Patate::SurfaceMesh::Vertex
ElementMesh<_Scalar, _Dim, _Chan>::addVertex(const Vector& pos)
{
    Vertex v = Patate::SurfaceMesh::addVertex();
    m_vPos[v] = pos;
    return v;
}

template < typename _Scalar, int _Dim, int _Chan >
void
ElementMesh<_Scalar, _Dim, _Chan>::reserve(
        unsigned nvertices, unsigned nedges, unsigned nfaces, unsigned nnodes)
{
    Patate::SurfaceMesh::reserve(nvertices, nedges, nfaces);
    m_nodes.reserve(nnodes);
}
