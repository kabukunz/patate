#ifndef _QMESH_H_
#define _QMESH_H_

#include <cassert>
#include <stdexcept>
#include <vector>
#include <fstream>

#include <Eigen/Dense>

class QvgParseError : public std::runtime_error
{
public:
    inline QvgParseError(const std::string& _what)
        : std::runtime_error(_what)
    {}
};

struct QuadraticTriangle
{
    inline const unsigned& vertex(unsigned _i) const
    {
        assert(_i < 3);
        return m_vertices[_i];
    }

    inline unsigned& vertex(unsigned _i)
    {
        assert(_i < 3);
        return m_vertices[_i];
    }

    inline const unsigned& vxNode(unsigned _i) const
    {
        assert(_i < 3);
        return m_nodes[_i];
    }

    inline unsigned& vxNode(unsigned _i)
    {
        assert(_i < 3);
        return m_nodes[_i];
    }

    // Edge i is the one opposed to vertex i
    inline const unsigned& edgeNode(unsigned _i) const
    {
        assert(_i < 3);
        return m_nodes[_i+3];
    }

    inline unsigned& edgeNode(unsigned _i)
    {
        assert(_i < 3);
        return m_nodes[_i+3];
    }

    unsigned m_vertices[3];
    unsigned m_nodes[6];
};

struct SingularQuadraticTriangle
{
    inline const unsigned& vertex(unsigned _i) const
    {
        assert(_i < 3);
        return m_vertices[_i];
    }

    inline unsigned& vertex(unsigned _i)
    {
        assert(_i < 3);
        return m_vertices[_i];
    }

    // The singular vertex is always vx 0. Node 1 & 2 correspond to vx 1 & 2.
    inline const unsigned& vxNode(unsigned _i) const
    {
        assert(_i < 4);
        return m_nodes[_i];
    }

    inline unsigned& vxNode(unsigned _i)
    {
        assert(_i < 4);
        return m_nodes[_i];
    }

    inline const unsigned& edgeNode(unsigned _i) const
    {
        assert(_i < 3);
        return m_nodes[_i+4];
    }

    inline unsigned& edgeNode(unsigned _i)
    {
        assert(_i < 3);
        return m_nodes[_i+4];
    }

    unsigned m_vertices[3];
    unsigned m_nodes[7];
};

class QMesh
{
public:
    typedef std::pair<Eigen::Vector2f, Eigen::Vector2f> Curve;

    typedef std::vector<Eigen::Vector2f> VertexList;
    typedef std::vector<Eigen::Vector4f> NodeList;
    typedef std::vector<Curve> CurveList;
    typedef std::vector<QuadraticTriangle> TriangleList;
    typedef std::vector<SingularQuadraticTriangle> SingularTriangleList;

public:
    QMesh() : 
        m_valid(false), m_vertices(), m_nodes(), m_curves(),
        m_triangles(), m_boundingBox(), m_singularTriangles()
    {}

    ~QMesh() {};

    void loadQvgFromFile(const std::string& _filename);
    void dumpQvg(std::ostream& _out);
    unsigned parseIndicesList(const std::string& _list, unsigned* _indices, unsigned _maxSize);

public:
    const bool& isValid() const { return m_valid; }
    unsigned nbVertices() const { return m_vertices.size(); }
    unsigned nbNodes() const { return m_nodes.size(); }
    unsigned nbCurves() const { return m_curves.size(); }
    unsigned nbTriangles() const { return m_triangles.size(); }
    unsigned nbSingularTriangles() const { return m_singularTriangles.size(); }

public:
    const bool& getValid() const { return m_valid; }
    bool& getValid() { return m_valid; }

    const VertexList& getVertices() const { return m_vertices; }
    VertexList& getVertices() { return m_vertices; }

    const NodeList& getNodes() const { return m_nodes; }
    NodeList& getNodes() { return m_nodes; }

    const CurveList& getCurves() const { return m_curves; }
    CurveList& getCurves() { return m_curves; }

    const TriangleList& getTriangles() const { return m_triangles; }
    TriangleList& getTriangles() { return m_triangles; }

    const SingularTriangleList& getSingularTriangles() const { return m_singularTriangles; }
    SingularTriangleList& getSingularTriangles() { return m_singularTriangles; }

    const Eigen::AlignedBox2f& getBoundingBox() const { return m_boundingBox; }
    Eigen::AlignedBox2f& getBoundingBox() { return m_boundingBox; }

protected:
    bool m_valid;

    VertexList m_vertices;
    NodeList m_nodes;
    CurveList m_curves;
    TriangleList m_triangles;
    SingularTriangleList m_singularTriangles;

    Eigen::AlignedBox2f m_boundingBox;
};

inline unsigned QMesh::parseIndicesList(const std::string& _list, unsigned* _indices, unsigned _maxSize)
{
    unsigned nbIndices = 1;
    std::string split(_list);
    for(std::string::iterator c = split.begin(); c != split.end(); ++c)
    {
        if(*c == '/')
        {
            *c = ' ';
            ++nbIndices;
        }
    }
    assert(nbIndices <= _maxSize);

    std::istringstream in(split);
    for(unsigned i = 0; i < nbIndices; ++i)
    {
        if(in.eof() || in.fail())
        {
            return 0;
        }
        in >> _indices[i];
    }

    if(!in.eof() || in.fail())
    {
        return 0;
    }

    return nbIndices;
}

#define QVG_CHECK_PARSE_ERROR() \
do\
{\
    in >> std::ws;\
    if(in.fail() || in.bad() || in.eof())\
    {\
        if(in.eof())\
            throw QvgParseError("Unexpected end of file.");\
        else\
            throw QvgParseError("Error while parsing. File probably malformed.");\
    }\
} while(false)

inline void QMesh::loadQvgFromFile(const std::string& _filename)
{
    getValid() = false;

    std::ifstream in(_filename.c_str());
    if(!in.good())
    {
        throw QvgParseError("Unable to read file.");
    }

    // Check header validity
    std::string tmpStr;
    in >> tmpStr;
    if(in.fail() && tmpStr != "qvg")
    {
        throw QvgParseError("Not a QVG file.");
    }
    QVG_CHECK_PARSE_ERROR();

    in >> tmpStr;
    if(!in.fail() && tmpStr != "1.0")
    {
        throw QvgParseError("Unsupported version "+tmpStr+".");
    }
    QVG_CHECK_PARSE_ERROR();

    //Check sizes
    unsigned nbVertices;
    unsigned nbNodes;
    unsigned nbCurves;
    unsigned nbTriangles;
    in >> nbVertices >> nbNodes >> nbCurves >> nbTriangles;
    QVG_CHECK_PARSE_ERROR();

    getVertices().resize(nbVertices);
    getNodes().resize(nbNodes);
    getCurves().resize(nbCurves);
    getTriangles().resize(nbTriangles);
    getSingularTriangles().resize(nbTriangles);

    //Read body
    // Vertices
    for(unsigned i = 0; i < nbVertices; ++i)
    {
        in >> tmpStr;
        if(!in.fail() && tmpStr != "v")
        {
            throw QvgParseError("Expected a vertex, got \""+tmpStr+"\".");
        }

        QMesh::VertexList& vertices = getVertices();

        in >> vertices[i](0) >> vertices[i](1);
        getBoundingBox().extend(vertices[i]);
        QVG_CHECK_PARSE_ERROR();
    }

    //Nodes
    for(unsigned i = 0; i < nbNodes; ++i)
    {
        in >> tmpStr;
        if(!in.fail() && tmpStr != "n")
        {
            throw QvgParseError("Expected a node, got \""+tmpStr+"\".");
        }

        QMesh::NodeList& nodes = getNodes();

        in >> nodes[i][0] >> nodes[i][1] >> nodes[i][2] >> nodes[i][3];
        QVG_CHECK_PARSE_ERROR();
    }

    // Curves
    for(unsigned i = 0; i < nbCurves; ++i)
    {
        in >> tmpStr;
        if(!in.fail() && tmpStr != "cc" && tmpStr != "cq")
        {
            throw QvgParseError("Expected a curve, got \""+tmpStr+"\".");
        }

        QMesh::CurveList& curves = getCurves();

        if(tmpStr == "cc")
        {
            in >> curves[i].first[0] >> curves[i].first[1] >> curves[i].second[0] >> curves[i].second[1];
        }
        else
        {
            in >> curves[i].first[0] >> curves[i].first[1];
            curves[i].second = curves[i].first;
        }
        QVG_CHECK_PARSE_ERROR();
    }

    // Faces
    unsigned tmpIndices[3];
    unsigned triangleIndex = 0;
    unsigned singularIndex = 0;

    QMesh::TriangleList& triangles = getTriangles();
    QMesh::SingularTriangleList& singularTriangles = getSingularTriangles();

    for(unsigned fi = 0; fi < nbTriangles; ++fi)
    {
        in >> tmpStr;
        if(!in.fail() && tmpStr != "f" && tmpStr != "fs")
        {
            throw QvgParseError("Expected a face, got \""+tmpStr+"\".");
        }
        QVG_CHECK_PARSE_ERROR();

        bool singular = (tmpStr == "fs");

        // read vertex and vertex nodes
        int singularVx = -1;

        for(unsigned vi = 0; vi < 3; ++vi)
        {
            in >> tmpStr;
            QVG_CHECK_PARSE_ERROR();

            unsigned read = parseIndicesList(tmpStr, tmpIndices, 3);
            if(!singular)
            {
                if(read != 2)
                {
                    throw QvgParseError("Unexpected number of vertex index.");
                }

                triangles[triangleIndex].vertex(vi) = tmpIndices[0];
                triangles[triangleIndex].vxNode(vi) = tmpIndices[1];
            }
            else
            {
                if(read != 2 && read != 3)
                {
                    throw QvgParseError("Unexpected number of vertex indices.");
                }

                singularTriangles[singularIndex].vertex(vi) = tmpIndices[0];
                singularTriangles[singularIndex].vxNode(vi) = tmpIndices[1];

                if(read == 3)
                {
                    if(singularVx != -1)
                    {
                        throw QvgParseError("Error: face with several singular vertices.");
                    }
                    singularTriangles[singularIndex].vxNode(3) = tmpIndices[2];
                    singularVx = vi;
                }
            }
        }

        if(singular && singularVx == -1)
        {
            throw QvgParseError("Error: singular triangle without singular node.");
        }

        // Read egde nodes
        for(unsigned ei = 0; ei < 3; ++ei)
        {
            in >> tmpStr;
            if(fi != nbTriangles - 1 || ei != 2)
            {
                QVG_CHECK_PARSE_ERROR();
            }
            else if(in.fail() || in.bad())
            {
                throw QvgParseError("Error while parsing. File probably malformed.");
            }

            unsigned read = parseIndicesList(tmpStr, tmpIndices, 3);

            if(read != 1 && read != 2)
            {
                throw QvgParseError("Unexpected number of edge indices.");
            }

            if(!singular)
            {
                triangles[triangleIndex].edgeNode(ei) = tmpIndices[0];
            }
            else
            {
                singularTriangles[singularIndex].edgeNode(ei) = tmpIndices[0];
            }
        }

        // Cyclic permutation so singular vertex is 0
        if(singularVx != -1)
        {
            std::rotate(singularTriangles[singularIndex].m_vertices, singularTriangles[singularIndex].m_vertices + singularVx, singularTriangles[singularIndex].m_vertices + 3);
            std::rotate(singularTriangles[singularIndex].m_nodes, singularTriangles[singularIndex].m_nodes + singularVx, singularTriangles[singularIndex].m_nodes + 3);
            std::rotate(singularTriangles[singularIndex].m_nodes + 4, singularTriangles[singularIndex].m_nodes + 4 + singularVx, singularTriangles[singularIndex].m_nodes + 7);
        }

        if(!singular)
        {
            ++triangleIndex;
        }
        else
        {
            ++singularIndex;
        }
    }

    triangles.resize(triangleIndex);
    singularTriangles.resize(singularIndex);

    getValid() = true;

    in >> std::ws;
    if(!in.eof())
    {
        throw QvgParseError("Error: expected end of file.");
    }
}

inline void QMesh::dumpQvg(std::ostream& _out)
{
    //assert(isValid());

    _out << "Vertices : " << nbVertices() << "\n";
    for(unsigned i = 0; i < nbVertices(); ++i)
    {
        _out << "  " << getVertices()[i].transpose() << "\n";
    }

    _out << "Nodes : " << nbNodes() << "\n";
    for(unsigned i = 0; i < nbNodes(); ++i)
    {
        _out << "  " << getNodes()[i].transpose() << "\n";
    }

    _out << "Edges : " << nbCurves() << "\n";
    for(unsigned i = 0; i < nbCurves(); ++i)
    {
        _out << "  " << getCurves()[i].first.transpose() << " / " << getCurves()[i].second.transpose() << "\n";
    }

    _out << "Triangles : " << nbTriangles() << "\n";
    for(unsigned i = 0; i < nbTriangles(); ++i)
    {
        _out    << "  "
            << getTriangles()[i].vertex(0) << " "
            << getTriangles()[i].vertex(1) << " "
            << getTriangles()[i].vertex(2) << " / "
            << getTriangles()[i].vxNode(0) << " "
            << getTriangles()[i].vxNode(1) << " "
            << getTriangles()[i].vxNode(2) << " / "
            << getTriangles()[i].edgeNode(0) << " "
            << getTriangles()[i].edgeNode(1) << " "
            << getTriangles()[i].edgeNode(2) << "\n";
    }

    _out << "Singular triangles: " << nbSingularTriangles() << "\n";
    for(unsigned i = 0; i < nbSingularTriangles(); ++i)
    {
        _out    << "  "
            << getSingularTriangles()[i].vertex(0) << " "
            << getSingularTriangles()[i].vertex(1) << " "
            << getSingularTriangles()[i].vertex(2) << " / "
            << getSingularTriangles()[i].vxNode(0) << " "
            << getSingularTriangles()[i].vxNode(1) << " "
            << getSingularTriangles()[i].vxNode(2) << " "
            << getSingularTriangles()[i].vxNode(3) << " / "
            << getSingularTriangles()[i].edgeNode(0) << " "
            << getSingularTriangles()[i].edgeNode(1) << " "
            << getSingularTriangles()[i].edgeNode(2) << "\n";
    }
}

#endif