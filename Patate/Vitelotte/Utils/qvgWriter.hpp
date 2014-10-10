
template < typename _Mesh >
void
QVGWriter<_Mesh>::write(std::ostream& _out) const
{
    typedef typename Mesh::NodeID NodeID;
    typedef typename Mesh::VertexIterator VertexIterator;
    typedef typename Mesh::FaceIterator FaceIterator;
    typedef typename Mesh::HalfedgeAroundFaceCirculator
            HalfedgeAroundFaceCirculator;

    assert(m_version == Version1_0);

    _out.imbue(std::locale::classic());

    int iOffset = 0;

    _out << "qvg 1.0\n";
    _out << m_mesh.nVertices() << " "
         << m_mesh.nNodes() << " "
         << "0 "  // << m_mesh.nCurves() << " "
         << m_mesh.nFaces() << "\n";

    for(VertexIterator vit = m_mesh.verticesBegin();
        vit != m_mesh.verticesEnd(); ++vit)
    {
        _out << "v " << m_mesh.position(*vit).transpose() << "\n";
    }

    for(unsigned i = 0; i < m_mesh.nNodes(); ++i)
    {
        if(m_mesh.isConstraint(i))
            _out << "n " << m_mesh.nodeValue(i).transpose() << "\n";
        else
            _out << "n void\n";
    }

    // TODO: Curve output.
//    for(unsigned i = 0; i < _mesh.nbCurves(); ++i)
//    {
//        _out << "  " << getCurves()[i].first.transpose() << " / " << getCurves()[i].second.transpose() << "\n";
//    }

    for(FaceIterator fit = m_mesh.facesBegin();
         fit != m_mesh.facesEnd(); ++fit)
    {
        _out << (m_mesh.isSingular(*fit)? "fs": "f");

        HalfedgeAroundFaceCirculator
                hit  = m_mesh.halfedges(*fit),
                hend = hit;
        do
        {
            NodeID to = m_mesh.toNode(*hit);
            NodeID from = m_mesh.fromNode(m_mesh.nextHalfedge(*hit));
            _out << " " << m_mesh.toVertex(*hit).idx() + iOffset
            << "/" << to + iOffset;
            if(from != to)
                _out << "/" << from + iOffset;
        }
        while(++hit != hend);

        ++hit; hend = hit;
        do
        {
            _out << " " << m_mesh.midNode(*hit) + iOffset;
        }
        while(++hit != hend);

        _out << "\n";
    }
}
