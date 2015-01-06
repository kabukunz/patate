#include "objReader.h"


namespace PatateCommon
{


bool defaultErrorCallback(const std::string& msg, void* ptr)
{
    std::cout << "Mvg parse error: " << msg << "\n";
    return true;
}


bool
OBJBaseReader::read(std::istream& in)
{
    m_lineNb = 0;
    m_error = false;
    m_line.reserve(200);
    std::string spec;

    in.imbue(std::locale::classic());

    if(in.bad())
    {
        error("Can not read input");
    }

    readLine(in);
    if(in.good() && !m_error)
        parseHeader(in);

    while(in.good() && !m_error)
    {
        // comment
        if(!m_line.empty() && m_line[0] != '#' && !std::isspace(m_line[0]))
        {
            m_lineStream >> spec;
            parseDefinition(spec, m_lineStream);
        }

        readLine(in);
    }

    return m_error;
}


bool
OBJBaseReader::readLine(std::istream& in)
{
    bool ok = std::getline(in, m_line).good();
    ++m_lineNb;
    m_lineStream.str(m_line);
    m_lineStream.seekg(0);
    return ok;
}


void
OBJBaseReader::parseIndiceList(const std::string& _list,
                               std::vector<unsigned>& _indices)
{
    _indices.clear();
    m_indicesStream.str(_list);
    m_indicesStream.seekg(0);
    while(m_indicesStream.good())
    {
        int i;
        if(m_indicesStream.peek() == '/' || m_indicesStream.peek() == 'x')
        {
            i = -1;
            m_indicesStream.get();
        }
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
    if(m_errorCallback)
    {
        m_error = m_errorCallback(msg, m_errorCallbackPtr);
    }
}


void
OBJBaseReader::warning(const std::string& msg)
{
    if(m_warningCallback)
    {
        m_error = m_warningCallback(msg, m_errorCallbackPtr);
    }
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
    {
        warning("Unknown spec: " + spec);
        return false;
    }
    return true;
}


}
