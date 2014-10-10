void
OBJBaseReader::read(std::istream& in)
{
    m_lineNb = 0;
    m_line.reserve(200);
    std::istringstream def;
    std::string spec;

    in.imbue(std::locale::classic());

    parseHeader(in);

    while(std::getline(in, m_line))
    {
        ++m_lineNb;

        // comment
        if(m_line.empty() || m_line[0] == '#' || std::isspace(m_line[0]))
            continue;

        def.str(m_line);
        def.seekg(0);
        def >> spec;
        parseDefinition(spec, def);
    }
}

void
OBJBaseReader::parseIndiceList(const std::string& _list,
                               std::vector<int>& _indices)
{
    _indices.clear();
    m_indicesStream.str(_list);
    m_indicesStream.seekg(0);
    while(m_indicesStream.good())
    {
        int i;
        if(m_indicesStream.peek() == '/')
            i = -1;
        else
            m_indicesStream >> i;
        _indices.push_back(i);

        if(m_indicesStream.good() && m_indicesStream.peek() == '/')
            m_indicesStream.get();
        else if(!m_indicesStream.eof())
            error("Unexpected character in indices list");
    }
    if(!m_indicesStream)
        error("Failed to read indices list");
}

void
OBJBaseReader::error(const std::string& msg)
{
    std::ostringstream out;
    out << "Error: " << m_lineNb << ": " << msg;
    throw std::runtime_error(out.str());
}



template < typename _Point >
OBJReader<_Point>::OBJReader(SurfaceMesh& mesh,
                             SurfaceMesh::VertexProperty<Point> positions)
    : m_mesh(mesh), m_vPos(positions)
{
}

template < typename _Point >
bool
OBJReader<_Point>::parseDefinition(const std::string& spec,
                                   std::istream& def)
{
    // vertex
    if (spec == "v")
    {
        Point p;
        for(unsigned i = 0; i < Point::SizeAtCompileTime; ++i)
            def >> p[i];
        if(!def) error("Failed to read vertex (not enough components ?)");
        SurfaceMesh::Vertex v = m_mesh.addVertex();
        m_vPos[v] = p;
    }
    // normal
//        else if (strncmp(s, "vn ", 3) == 0)
//        {
//            if (sscanf(s, "vn %f %f %f", &x, &y, &z))
//            {
//                // problematic as it can be either a vertex property when interpolated
//                // or a halfedge property for hard edges
//            }
//        }

    // texture coordinate
//        else if (strncmp(s, "vt ", 3) == 0)
//        {
//            if (sscanf(s, "vt %f %f", &x, &y))
//            {
//                z=1;
//                all_tex_coords.push_back(Texture_coordinate(x,y,z));
//            }
//        }

    // face
    else if (spec == "f")
    {
        m_fVertices.clear();

        def >> std::ws;

        while(def.good())
        {
            // TODO: Use parseIndiceList to read indices
            unsigned idx;
            def >> idx;
            m_fVertices.push_back(SurfaceMesh::Vertex(idx - 1));
        }

        m_mesh.addFace(m_fVertices);
    }
    else
        return false;
    return true;
}


