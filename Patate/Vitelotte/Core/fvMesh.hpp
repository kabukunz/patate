
template < typename _Scalar, int _Dim, int _Chan >
FVMesh<_Scalar, _Dim, _Chan>::FVMesh()
{
    m_vFlatGrad = Base::template addVertexProperty<bool>("v:flatGrad", false);
    m_hGradNode = Base::template addHalfedgeProperty<NodeID>("h:gradNode", InvalidNodeID);
}

template < typename _Scalar, int _Dim, int _Chan >
template < typename OtherScalar >
FVMesh<_Scalar, _Dim, _Chan>&
FVMesh<_Scalar, _Dim, _Chan>::operator=(
        const FVMesh<OtherScalar, _Dim, _Chan>& rhs)
{
    if(&rhs != this)
    {
        // FIXME: SurfaceMesh's operator= wont work with properties of different types.
        Base::operator=(rhs);

        m_vFlatGrad = Base::template vertexProperty<bool>("v:flatGrad");
        m_hGradNode = Base::template halfedgeProperty<NodeID>("h:gradNode");
    }
    return *this;
}

template < typename _Scalar, int _Dim, int _Chan >
void
FVMesh<_Scalar, _Dim, _Chan>::compactNodes()
{
    internal::compactNodes(*this);
}

template < typename _Scalar, int _Dim, int _Chan >
template < typename Marked >
void
FVMesh<_Scalar, _Dim, _Chan>::markNodes(Halfedge h, Marked& marked) const
{
    Base::markNodes(h, marked);
    marked[gradientNode(h)] = 1;
}

template < typename _Scalar, int _Dim, int _Chan >
template < typename Map >
void
FVMesh<_Scalar, _Dim, _Chan>::remapNodes(Halfedge h, Map& map)
{
    Base::remapNodes(h, map);
    gradientNode(h) = map[gradientNode(h)];
}

