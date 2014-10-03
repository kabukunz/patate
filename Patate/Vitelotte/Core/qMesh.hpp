

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

