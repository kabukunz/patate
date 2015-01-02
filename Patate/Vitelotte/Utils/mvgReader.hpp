#include "mvgReader.h"


namespace Vitelotte
{


template < typename _Mesh >
MVGReader<_Mesh>::MVGReader(Mesh& mesh)
    : PatateCommon::OBJReader<Vector>(mesh, mesh.positionProperty())
{
    m_faceIndices.reserve(3);
}


template < typename _Mesh >
void
MVGReader<_Mesh>::parseHeader(std::istream& in)
{
    Base::m_lineStream >> m_tmp;
    if(!in) error("Failed to read header");
    if(m_tmp != "mvg") error("Missing mvg header");

    Base::m_lineStream >> m_tmp;
    if(!in) error("Failed to read header");
    if(m_tmp != "1.0") error("Unsuported version");

    std::string cmd;
    unsigned dim = 2;
    unsigned params = 4;
    unsigned attributes = 0;
    unsigned nVert = 1024;
    unsigned nNode = 1024;
    unsigned nFace = 1024;
    while(true)
    {
        Base::readLine(in);
        Base::m_lineStream >> cmd;

        if(cmd == "dim")
            Base::m_lineStream >> dim;
        else if(cmd == "parameters")
            Base::m_lineStream >> params;
        else if(cmd == "linear")
            attributes = Mesh::Linear;
        else if(cmd == "quadratic")
            attributes = Mesh::Quadratic;
        else if(cmd == "morley")
            attributes = Mesh::Morley;
        else if(cmd == "fv")
            attributes = Mesh::FV;
        else if(cmd == "mesh")
            Base::m_lineStream >>attributes;
        else if(cmd == "vertices")
            Base::m_lineStream >> nVert;
        else if(cmd == "nodes")
            Base::m_lineStream >> nNode;
        else if(cmd == "faces")
            Base::m_lineStream >> nFace;
        else
            break;

        if(!in || !Base::m_lineStream) error("Failed to read header");
    }
    Base::m_lineStream.seekg(0);

    Mesh& m = static_cast<Mesh&>(m_mesh);
    m.clear();
    // TODO: set / check dims and parameters
    m.setAttributes(attributes);
    m.reserve(nVert, nVert+nFace, nFace, nNode);
}


template < typename _Mesh >
bool
MVGReader<_Mesh>::parseDefinition(const std::string& spec,
                                   std::istream& def)
{
    typedef typename Mesh::Node Node;

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
        int nodes[12];

        for(int i = 0; i < 3; ++i)
        {
            def >> m_tmp;
            parseIndiceList(m_tmp, m_faceIndices);
            if(m_faceIndices.size() < 1 || m_faceIndices.size() > 3)
                error("Invalid number of indices");

            m_fVertices.push_back(
                        typename Mesh::Vertex(m_faceIndices[0] - iOffset));

            if(m_faceIndices.size() > 1)
            {
                nodes[2*i + 0] = m_faceIndices[1];
                nodes[2*i + 1] = m_faceIndices.back();
            }
        }

        // mid nodes
        unsigned nEAttrs = m.hasEdgeValue() + m.hasEdgeGradient();
        if(nEAttrs)
        {
            def >> m_tmp;
            if(m_tmp != "-")
                error("Only triangles meshes are supported");

            for(int i = 0; i < 3; ++i)
            {
                def >> m_tmp;
                parseIndiceList(m_tmp, m_faceIndices);
                if(m_faceIndices.size() != nEAttrs)
                    error("Invalid number of indices");

                if(m.hasEdgeValue())
                    nodes[6+i] = m_faceIndices.front();
                if(m.hasEdgeGradient())
                    nodes[9+i] = m_faceIndices.back();
            }
        }

        typename Mesh::Face f = m.addFace(m_fVertices);

        typename Mesh::HalfedgeAroundFaceCirculator hit = m.halfedges(f);
        for(int i = 0; i < 3; ++i)
        {
            if(m.hasVertexValue())
                m.vertexValueNode(*hit) = Node(nodes[2*i] - iOffset);
            ++hit;
            if(m.hasVertexFromValue())
                m.vertexFromValueNode(*hit) = Node(nodes[2*i + 1] - iOffset);
            if(m.hasEdgeValue())
                m.edgeValueNode(*hit) = Node(nodes[6 + (i+2)%3] - iOffset);
            if(m.hasEdgeGradient())
                m.edgeGradientNode(*hit) = Node(nodes[9 + (i+2)%3] - iOffset);
        }
    }
    else
        return false;
    return true;
}


template < typename Mesh >
void readMvg(std::istream& in, Mesh& mesh)
{
    MVGReader<Mesh> reader(mesh);
    reader.read(in);
}

template < typename Mesh >
void readMvgFromFile(const std::string& filename, Mesh& mesh)
{
    std::ifstream in(filename.c_str());
    readMvg(in, mesh);
}


}  // namespace Vitelotte
