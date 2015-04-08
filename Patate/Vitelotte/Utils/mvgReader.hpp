/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "mvgReader.h"


namespace Vitelotte
{


#define PTT_ERROR_IF(_cond, _msg, _ret) do { if(_cond) { error(_msg); return _ret; } } while(false)
#define PTT_RETURN_IF_ERROR(_ret) do { if(m_error) { return _ret; } } while(false)


template < typename _Mesh >
MVGReader<_Mesh>::MVGReader()
{
    m_faceIndices.reserve(3);
}


template < typename _Mesh >
void
MVGReader<_Mesh>::parseHeader(std::istream& in, Mesh& mesh)
{
    m_lineStream >> m_tmp;
    if(!in) error("Failed to read header");
    if(m_tmp != "mvg") error("Missing mvg header");

    m_lineStream >> m_tmp;
    if(!in) error("Failed to read header");
    if(m_tmp != "1.0") error("Unsuported version");

    std::string cmd;
    unsigned nDims = 2;
    unsigned nCoeffs = 4;
    unsigned attributes = 0;
    unsigned nVert = 1024;
    unsigned nNode = 1024;
    unsigned nFace = 1024;
    while(true)
    {
        readLine(in);
        m_lineStream >> cmd;

        if(cmd == "dim")
            m_lineStream >> nDims;
        else if(cmd == "parameters" || cmd == "coefficients")
            m_lineStream >> nCoeffs;
        else if(cmd == "linear")
            attributes = Mesh::LINEAR_FLAGS;
        else if(cmd == "quadratic")
            attributes = Mesh::QUADRATIC_FLAGS;
        else if(cmd == "morley")
            attributes = Mesh::MORLEY_FLAGS;
        else if(cmd == "fv")
            attributes = Mesh::FV_FLAGS;
        else if(cmd == "attributes")
        {
            m_lineStream >> cmd;
            if(cmd == "none")
                attributes = 0;
            else if(cmd == "linear")
                attributes = Mesh::LINEAR_FLAGS;
            else if(cmd == "quadratic")
                attributes = Mesh::QUADRATIC_FLAGS;
            else if(cmd == "morley")
                attributes = Mesh::MORLEY_FLAGS;
            else if(cmd == "fv")
                attributes = Mesh::FV_FLAGS;
        }
        else if(cmd == "mesh")
            m_lineStream >> attributes;
        else if(cmd == "vertices")
            m_lineStream >> nVert;
        else if(cmd == "nodes")
            m_lineStream >> nNode;
        else if(cmd == "faces")
            m_lineStream >> nFace;
        else
            break;

        if(!in || !m_lineStream) error("Failed to read header");
    }
    m_lineStream.seekg(0);

    if(int(Mesh::DimsAtCompileTime) != int(Dynamic) && nDims != mesh.nDims())
    {
        error("Invalid number of dimensions.");
        return;
    }
    if(int(Mesh::CoeffsAtCompileTime) != int(Dynamic) && nCoeffs != mesh.nCoeffs())
    {
        error("Invalid number of coefficients.");
        return;
    }

    mesh.clear();
    mesh.setAttributes(attributes);
    mesh.setNDims(nDims);
    mesh.setNCoeffs(nCoeffs);
    mesh.reserve(nVert, nVert+nFace, nFace, nNode);

    m_vector.resize(nDims);
    m_value.resize(nCoeffs);
    m_gradient.resize(nCoeffs, nDims);
}


template < typename _Mesh >
bool
MVGReader<_Mesh>::parseDefinition(const std::string& spec,
                                   std::istream& def, Mesh& mesh)
{
    typedef typename Mesh::Node Node;

    int iOffset = 0;

    // vertex
    if(spec == "v")
    {
        parseVector(def); PTT_RETURN_IF_ERROR(true);
        if(!def.eof()) warning("Too much components.");
        mesh.addVertex(m_vector);
    }

    // nodes
    else if(spec == "n")
    {
        parseValueWithVoid(def, mesh); PTT_RETURN_IF_ERROR(true);
        if(!def.eof()) warning("Too much components.");
        mesh.addNode(m_value);
    }

    // face
    else if(spec == "f")
    {
        m_fVertices.clear();

        def >> std::ws;
        int nodes[12];

        for(int i = 0; i < 3; ++i)
        {
            def >> m_tmp;
            parseIndicesList(m_tmp, m_faceIndices);
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
        unsigned nEAttrs = mesh.hasEdgeValue() + mesh.hasEdgeGradient();
        if(nEAttrs)
        {
            def >> m_tmp;
            if(m_tmp != "-")
                error("Only triangles meshes are supported");

            for(int i = 0; i < 3; ++i)
            {
                def >> m_tmp;
                parseIndicesList(m_tmp, m_faceIndices);
                if(m_faceIndices.size() != nEAttrs)
                    error("Invalid number of indices");

                if(mesh.hasEdgeValue())
                    nodes[6+i] = m_faceIndices.front();
                if(mesh.hasEdgeGradient())
                    nodes[9+i] = m_faceIndices.back();
            }
        }

        typename Mesh::Face f = mesh.addFace(m_fVertices);

        typename Mesh::HalfedgeAroundFaceCirculator hit = mesh.halfedges(f);
        for(int i = 0; i < 3; ++i)
        {
            if(mesh.hasToVertexValue())
                mesh.toVertexValueNode(*hit) = Node(nodes[2*i] - iOffset);
            ++hit;
            if(mesh.hasFromVertexValue())
                mesh.fromVertexValueNode(*hit) = Node(nodes[2*i + 1] - iOffset);
            if(mesh.hasEdgeValue())
                mesh.edgeValueNode(*hit) = Node(nodes[6 + (i+2)%3] - iOffset);
            if(mesh.hasEdgeGradient())
                mesh.edgeGradientNode(*hit) = Node(nodes[9 + (i+2)%3] - iOffset);
        }
    }

    else if(spec == "vgc")
    {
        unsigned vxIdx;
        def >> vxIdx;
        PTT_ERROR_IF(!def || vxIdx >= mesh.nVertices(), "Invalid vertex index", true);

        parseGradient(def); PTT_RETURN_IF_ERROR(true);
        if(!def.eof()) warning("Too much components.");

        mesh.setGradientConstraint(Vertex(vxIdx), m_gradient);
    }

    // Unknown element type.
    else
    {
        warning("Unknown spec: " + spec);
        return false;
    }
    return true;
}


template < typename _Mesh >
void
MVGReader<_Mesh>::parseValue(std::istream& in) {
    for(unsigned i = 0; i < m_value.size(); ++i) {
        in >> m_value(i);
    }
    PTT_ERROR_IF(!in, "Invalid value specification",);
    in >> std::ws;
}


template < typename _Mesh >
void
MVGReader<_Mesh>::parseValueWithVoid(std::istream& in, Mesh& mesh) {
    in >> std::ws;
    PTT_ERROR_IF(!in.good(), "Invalid value specification",);
    if(std::isalpha(in.peek()))
    {
        in >> m_tmp;
        PTT_ERROR_IF(m_tmp != "void", "Invalid value specification",);
        m_value = mesh.unconstrainedValue();
    }
    else
    {
        parseValue(in); PTT_RETURN_IF_ERROR();
    }
    in >> std::ws;
}


template < typename _Mesh >
void
MVGReader<_Mesh>::parseGradient(std::istream& in) {
    for(unsigned i = 0; i < m_gradient.size(); ++i) {
        in >> m_gradient(i);
    }
    PTT_ERROR_IF(!in, "Invalid gradient specification",);
    in >> std::ws;
}


template < typename Mesh >
bool readMvg(std::istream& in, Mesh& mesh)
{
    MVGReader<Mesh> reader;
    return reader.read(in, mesh);
}


template < typename Mesh >
bool readMvgFromFile(const std::string& filename, Mesh& mesh)
{
    std::ifstream in(filename.c_str());
    return readMvg(in, mesh);
}


#undef PTT_ERROR_IF
#undef PTT_RETURN_IF_ERROR

}  // namespace Vitelotte
