
inline void QMesh::clear()
{
    m_valid = true;
    m_vertices.clear();
    m_nodes.clear();
    m_curves.clear();
    m_triangles.clear();
    m_singularTriangles.clear();
    m_boundingBox.setEmpty();
}
