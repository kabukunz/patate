
inline void QVGWriter::write(const QMesh& _mesh, std::ostream& _out) const
{
    assert(m_version == Version1_0);

    const QMesh::VertexList& vertices = _mesh.getVertices();
    const QMesh::NodeList& nodes = _mesh.getNodes();
    const QMesh::CurveList& curves = _mesh.getCurves();
    const QMesh::TriangleList& triangles = _mesh.getTriangles();
    const QMesh::SingularTriangleList& singularTriangles = _mesh.getSingularTriangles();

    _out << "qvg 1.0\n";
    _out << _mesh.nbVertices() << " "
         << _mesh.nbNodes() << " "
         << _mesh.nbCurves() << " "
         << _mesh.nbTriangles() + _mesh.nbSingularTriangles() << "\n";

    for(unsigned i = 0; i < _mesh.nbVertices(); ++i)
    {
        _out << "v " << vertices[i].transpose() << "\n";
    }

    for(unsigned i = 0; i < _mesh.nbNodes(); ++i)
    {
        _out << "n " << nodes[i].transpose() << "\n";
    }

    // TODO: Curve output.
//    for(unsigned i = 0; i < _mesh.nbCurves(); ++i)
//    {
//        _out << "  " << getCurves()[i].first.transpose() << " / " << getCurves()[i].second.transpose() << "\n";
//    }

    for(unsigned i = 0; i < _mesh.nbTriangles(); ++i)
    {
        _out << "f "
             << triangles[i].vertex(0) << "/"
             << triangles[i].vxNode(0) << " "
             << triangles[i].vertex(1) << "/"
             << triangles[i].vxNode(1) << " "
             << triangles[i].vertex(2) << "/"
             << triangles[i].vxNode(2) << " "
             << triangles[i].edgeNode(0) << " "
             << triangles[i].edgeNode(1) << " "
             << triangles[i].edgeNode(2) << "\n";
    }

    for(unsigned i = 0; i < _mesh.nbSingularTriangles(); ++i)
    {
        _out << "fs "
             << singularTriangles[i].vertex(0) << "/"
             << singularTriangles[i].vxNode(0) << "/"
             << singularTriangles[i].vxNode(3) << " "
             << singularTriangles[i].vertex(1) << "/"
             << singularTriangles[i].vxNode(1) << " "
             << singularTriangles[i].vertex(2) << "/"
             << singularTriangles[i].vxNode(2) << " "
             << singularTriangles[i].edgeNode(0) << " "
             << singularTriangles[i].edgeNode(1) << " "
             << singularTriangles[i].edgeNode(2) << "\n";
    }
}
