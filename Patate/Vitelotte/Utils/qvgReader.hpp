
#define PATATE_QVG_CHECK_PARSE_ERROR() \
do\
{\
    _in >> std::ws;\
    if(_in.fail() || _in.bad() || _in.eof())\
    {\
        if(_in.eof())\
            throw QVGReadError("Unexpected end of file.");\
        else\
            throw QVGReadError("Error while parsing. File probably malformed.");\
    }\
} while(false)

inline void QVGReader::read(QMesh& _mesh, std::istream& _in) const
{
    _mesh.getValid() = false;

    // Check header validity
    std::string tmpStr;
    _in >> tmpStr;
    if(!_in.fail() && tmpStr != "qvg")
    {
        throw QVGReadError("Not a QVG file.");
    }
    PATATE_QVG_CHECK_PARSE_ERROR();

    _in >> tmpStr;
    if(!_in.fail() && tmpStr != "1.0")
    {
        throw QVGReadError("Unsupported version "+tmpStr+".");
    }
    PATATE_QVG_CHECK_PARSE_ERROR();

    //Check sizes
    unsigned nbVertices;
    unsigned nbNodes;
    unsigned nbCurves;
    unsigned nbTriangles;
    _in >> nbVertices >> nbNodes >> nbCurves >> nbTriangles;
    PATATE_QVG_CHECK_PARSE_ERROR();

    QMesh::VertexList& vertices = _mesh.getVertices();
    QMesh::NodeList& nodes = _mesh.getNodes();
    QMesh::CurveList& curves = _mesh.getCurves();
    QMesh::TriangleList& triangles = _mesh.getTriangles();
    QMesh::SingularTriangleList& singularTriangles = _mesh.getSingularTriangles();

    vertices.resize(nbVertices);
    nodes.resize(nbNodes);
    curves.resize(nbCurves);
    triangles.resize(nbTriangles);
    singularTriangles.resize(nbTriangles);

    //Read body
    // Vertices
    for(unsigned i = 0; i < nbVertices; ++i)
    {
        _in >> tmpStr;
        if(!_in.fail() && tmpStr != "v")
        {
            throw QVGReadError("Expected a vertex, got \""+tmpStr+"\".");
        }

        _in >> vertices[i](0) >> vertices[i](1);
        _mesh.getBoundingBox().extend(vertices[i]);
        PATATE_QVG_CHECK_PARSE_ERROR();
    }

    //Nodes
    for(unsigned i = 0; i < nbNodes; ++i)
    {
        _in >> tmpStr;
        if(!_in.fail() && tmpStr != "n")
        {
            throw QVGReadError("Expected a node, got \""+tmpStr+"\".");
        }

        _in >> nodes[i][0] >> nodes[i][1] >> nodes[i][2] >> nodes[i][3];
        PATATE_QVG_CHECK_PARSE_ERROR();
    }

    // Curves
    for(unsigned i = 0; i < nbCurves; ++i)
    {
        _in >> tmpStr;
        if(!_in.fail() && tmpStr != "cc" && tmpStr != "cq")
        {
            throw QVGReadError("Expected a curve, got \""+tmpStr+"\".");
        }

        if(tmpStr == "cc")
        {
            _in >> curves[i].first[0] >> curves[i].first[1] >> curves[i].second[0] >> curves[i].second[1];
        }
        else
        {
            _in >> curves[i].first[0] >> curves[i].first[1];
            curves[i].second = curves[i].first;
        }
        PATATE_QVG_CHECK_PARSE_ERROR();
    }

    // Faces
    unsigned tmpIndices[3];
    unsigned triangleIndex = 0;
    unsigned singularIndex = 0;

    for(unsigned fi = 0; fi < nbTriangles; ++fi)
    {
        _in >> tmpStr;
        if(!_in.fail() && tmpStr != "f" && tmpStr != "fs")
        {
            throw QVGReadError("Expected a face, got \""+tmpStr+"\".");
        }
        PATATE_QVG_CHECK_PARSE_ERROR();

        bool singular = (tmpStr == "fs");

        // read vertex and vertex nodes
        int singularVx = -1;

        for(unsigned vi = 0; vi < 3; ++vi)
        {
            _in >> tmpStr;
            PATATE_QVG_CHECK_PARSE_ERROR();

            unsigned read = parseIndicesList(tmpStr, tmpIndices, 3);
            if(!singular)
            {
                if(read != 2)
                {
                    throw QVGReadError("Unexpected number of vertex indices.");
                }

                triangles[triangleIndex].vertex(vi) = tmpIndices[0];
                triangles[triangleIndex].vxNode(vi) = tmpIndices[1];
            }
            else
            {
                if(read != 2 && read != 3)
                {
                    throw QVGReadError("Unexpected number of vertex indices.");
                }

                singularTriangles[singularIndex].vertex(vi) = tmpIndices[0];
                singularTriangles[singularIndex].vxNode(vi) = tmpIndices[1];

                if(read == 3)
                {
                    if(singularVx != -1)
                    {
                        throw QVGReadError("Error: face with several singular vertices.");
                    }
                    singularTriangles[singularIndex].vxNode(3) = tmpIndices[2];
                    singularVx = vi;
                }
            }
        }

        if(singular && singularVx == -1)
        {
            throw QVGReadError("Error: singular triangle without singular node.");
        }

        // Read egde nodes
        for(unsigned ei = 0; ei < 3; ++ei)
        {
            _in >> tmpStr;
            if(fi != nbTriangles - 1 || ei != 2)
            {
                PATATE_QVG_CHECK_PARSE_ERROR();
            }
            else if(_in.fail() || _in.bad())
            {
                throw QVGReadError("Error while parsing. File probably malformed.");
            }

            unsigned read = parseIndicesList(tmpStr, tmpIndices, 3);

            if(read != 1 && read != 2)
            {
                throw QVGReadError("Unexpected number of edge indices.");
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

    _mesh.getValid() = true;
}

inline unsigned QVGReader::parseIndicesList(
        const std::string& _list, unsigned* _indices, unsigned _maxSize) const
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

#undef PATATE_QVG_CHECK_PARSE_ERROR
