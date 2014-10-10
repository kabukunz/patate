template < typename _Mesh >
QVGReader<_Mesh>::QVGReader(Mesh& mesh)
    : Patate::OBJReader<Vector>(mesh, mesh.vPosProperty())
{
    m_faceIndices.reserve(3);
}

template < typename _Mesh >
void
QVGReader<_Mesh>::parseHeader(std::istream& in)
{
    in >> m_tmp;
    if(!in) error("Failed to read header");
    if(m_tmp != "qvg") error("Missing qvg header");

    in >> m_tmp;
    if(!in) error("Failed to read header");
    if(m_tmp != "1.0") error("Unsuported version");

    unsigned nVert, nNode, nCurve, nFace;
    in >> nVert >> nNode >> nCurve >> nFace;
    if(!in) error("Failed to read header");

    Mesh& m = static_cast<Mesh&>(m_mesh);
    m.clear();
    m.reserve(nVert, nCurve, nFace, nNode);
}

template < typename _Mesh >
bool
QVGReader<_Mesh>::parseDefinition(const std::string& spec,
                                   std::istream& def)
{
    Mesh& m = static_cast<Mesh&>(m_mesh);

    int iOffset = 0;

    // vertex
    if(spec == "v")
    {
        Base::parseDefinition(spec, def);
    }

    // nodes
    else if(spec == "n")
    {
        NodeValue n;
        def >> std::ws;
        if(def.peek() == 'v')
            n = Mesh::UnconstrainedNode;
        else
        {
            for(unsigned i = 0; i < NodeValue::SizeAtCompileTime; ++i)
                def >> n[i];
            if(!def) error("Failed to read node (not enough components ?)");
        }
        m.addNode(n);
    }

    // face
    else if(spec == "f" || spec == "fs")
    {
        m_fVertices.clear();

        def >> std::ws;
        int nodes[6];
        int singular = 0;

        for(int i = 0; i < 3; ++i)
        {
            def >> m_tmp;
            parseIndiceList(m_tmp, m_faceIndices);
            if(m_faceIndices.size() < 2 || m_faceIndices.size() > 3)
                error("Invalid number of indices");

            m_fVertices.push_back(
                        typename Mesh::Vertex(m_faceIndices[0] - iOffset));
            nodes[2*i] = m_faceIndices[1];
            nodes[2*i + 1] = m_faceIndices.back();
        }

        typename Mesh::Face f = m.addFace(m_fVertices);

        typename Mesh::HalfedgeAroundFaceCirculator hit  = m.halfedges(f);
        for(int i = 0; i < 3; ++i)
        {
            int mid;
            def >> mid;
            if(!def) error("Missing edge node");

            m.toNode(*hit) = nodes[2*i] - iOffset;
            ++hit;
            m.midNode(*hit) = mid - iOffset;
            m.fromNode(*hit) = nodes[2*i + 1] - iOffset;
        }
    }
    else
        return false;
    return true;
}
