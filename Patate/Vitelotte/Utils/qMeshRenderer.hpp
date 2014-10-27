

inline bool QMeshRenderer::init(QMesh* _qMesh)
{
    setQMesh(_qMesh);

    if(m_pQMesh && m_pQMesh->isValid())
    {
        if(loadShaders())
        {
            glGenVertexArrays(1, &m_vao);
            glBindVertexArray(m_vao);

            return initGl();
        }
    }

    return false;
}

inline bool QMeshRenderer::loadShaders()
{
    PATATE_GLCheckError();

    m_pTriangleProgram = new Patate::Shader();
    if(!m_pTriangleProgram->Init())
    {
        return false;
    }

    bool bRes = true;

    bRes &= m_pTriangleProgram->AddShader(GL_VERTEX_SHADER, vert_common_glsl);
    bRes &= m_pTriangleProgram->AddShader(GL_GEOMETRY_SHADER, geom_common_glsl);
    bRes &= m_pTriangleProgram->AddShader(GL_FRAGMENT_SHADER, frag_common_glsl);
    bRes &= m_pTriangleProgram->AddShader(GL_FRAGMENT_SHADER, frag_triangle_glsl);
    assert(bRes);

    bRes &= m_pTriangleProgram->Finalize();
    assert(bRes);


    m_pSingularProgram = new Patate::Shader();
    if(!m_pSingularProgram->Init())
    {
        return false;
    }

    bRes &= m_pSingularProgram->AddShader(GL_VERTEX_SHADER, vert_common_glsl);
    bRes &= m_pSingularProgram->AddShader(GL_GEOMETRY_SHADER, geom_common_glsl);
    bRes &= m_pSingularProgram->AddShader(GL_FRAGMENT_SHADER, frag_common_glsl);
    bRes &= m_pSingularProgram->AddShader(GL_FRAGMENT_SHADER, frag_singular_glsl);
    assert(bRes);

    bRes &= m_pSingularProgram->Finalize();
    assert(bRes);

    return PATATE_GLCheckError();;
}

inline bool QMeshRenderer::initGl()
{
    std::vector<unsigned> triangleIndices;
    std::vector<unsigned> singularIndices;
    QMesh::NodeList triangleNodes;
    QMesh::NodeList singularNodes;

    triangleIndices.reserve(m_pQMesh->nbTriangles() * 3);
    singularIndices.reserve(m_pQMesh->nbSingularTriangles() * 3);
    triangleNodes.reserve(m_pQMesh->nbTriangles() * 6);
    singularNodes.reserve(m_pQMesh->nbSingularTriangles() * 7);

    for(unsigned i = 0; i < m_pQMesh->nbTriangles(); ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            triangleIndices.push_back(m_pQMesh->getTriangles()[i].m_vertices[j]);
        }
        for(int j = 0; j < 6; ++j)
        {
            triangleNodes.push_back(m_pQMesh->getNodes()[m_pQMesh->getTriangles()[i].m_nodes[j]]);
        }
    }

    for(unsigned i = 0; i < m_pQMesh->nbSingularTriangles(); ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            singularIndices.push_back(m_pQMesh->getSingularTriangles()[i].m_vertices[j]);
        }
        for(int j = 0; j < 7; ++j)
        {
            singularNodes.push_back(m_pQMesh->getNodes()[m_pQMesh->getSingularTriangles()[i].m_nodes[j]]);
        }
    }

    if(!m_verticesBuffer)
    {
        glGenBuffers(1, &m_verticesBuffer);
    }
    glBindBuffer(GL_ARRAY_BUFFER, m_verticesBuffer);
    glBufferData(GL_ARRAY_BUFFER, m_pQMesh->nbVertices() * sizeof(Eigen::Vector2f), &(m_pQMesh->getVertices()[0]), GL_STATIC_DRAW);

    if(!m_triangleIndicesBuffer)
    {
        glGenBuffers(1, &m_triangleIndicesBuffer);
    }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_triangleIndicesBuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_pQMesh->nbTriangles() * 3 * sizeof(unsigned), &triangleIndices[0], GL_STATIC_DRAW);

    if(m_pQMesh->nbSingularTriangles() > 0)
    {
        if(!m_singularIndicesBuffer)
        {
            glGenBuffers(1, &m_singularIndicesBuffer);
        }
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_singularIndicesBuffer);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_pQMesh->nbSingularTriangles() * 3 * sizeof(unsigned), &singularIndices[0], GL_STATIC_DRAW);
    }

    if(!m_triangleNodesBuffer)
    {
        glGenBuffers(1, &m_triangleNodesBuffer);
    }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_triangleNodesBuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, triangleNodes.size() * sizeof(Eigen::Vector4f), &triangleNodes[0], GL_STATIC_DRAW);

    if(singularNodes.size() > 0)
    {
        if(!m_singularNodesBuffer)
        {
            glGenBuffers(1, &m_singularNodesBuffer);
        }
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_singularNodesBuffer);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, singularNodes.size() * sizeof(Eigen::Vector4f), &singularNodes[0], GL_STATIC_DRAW);
    }

    if(!m_triangleNodesTexture)
    {
        glGenTextures(1, &m_triangleNodesTexture);
    }
    glBindTexture(GL_TEXTURE_BUFFER, m_triangleNodesTexture);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, m_triangleNodesBuffer);

    if(m_singularNodesBuffer)
    {
        if(!m_singularNodesTexture)
        {
            glGenTextures(1, &m_singularNodesTexture);
        }
        glBindTexture(GL_TEXTURE_BUFFER, m_singularNodesTexture);
        glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, m_singularNodesBuffer);
    }

    return (glGetError() == GL_NO_ERROR);
}

inline void QMeshRenderer::renderTriangles(GLuint _shader, bool _singular)
{
    if(m_pQMesh && m_pQMesh->isValid())
    {
        if(_singular && !(m_pQMesh->nbSingularTriangles() > 0))
        {
            return;
        }

        glUseProgram(_shader);

        GLint verticesLoc = glGetAttribLocation(_shader, "vx_position");

        if(verticesLoc >= 0)
        {
            glEnableVertexAttribArray(verticesLoc);
            glBindBuffer(GL_ARRAY_BUFFER, m_verticesBuffer);
            glVertexAttribPointer(verticesLoc, 2, GL_FLOAT, false, sizeof(Eigen::Vector2f), 0);
        }

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_BUFFER, _singular ? m_singularNodesTexture : m_triangleNodesTexture);
        glUniform1i(glGetUniformLocation(_shader, "nodes"), 0);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _singular ? m_singularIndicesBuffer : m_triangleIndicesBuffer);
        glDrawElements(GL_TRIANGLES, m_pQMesh->nbTriangles() * 3, GL_UNSIGNED_INT, 0);

        if(verticesLoc >= 0)
        {
            glDisableVertexAttribArray(verticesLoc);
        }
    }
}

inline void QMeshRenderer::render(Eigen::Matrix4f& _viewMatrix, float _zoom, float _pointRadius, float _lineWidth, bool _showShaderWireframe)
{
    for(int pass = 0; pass < 2; ++pass)
    {
        Patate::Shader* program = (pass == 0) ? m_pTriangleProgram : m_pSingularProgram;

        if(program)
        {
            program->Enable();

            GLuint viewMatrixLoc = program->GetUniformLocation("viewMatrix");
            if(viewMatrixLoc >= 0)
            {
                glUniformMatrix4fv(viewMatrixLoc, 1, false, _viewMatrix.data());
            }

            GLint wireLoc = program->GetUniformLocation("showWireframe");
            if(wireLoc >=0 )
            {
                glUniform1i(wireLoc, _showShaderWireframe);
            }

            GLuint zoomLoc = program->GetUniformLocation("zoom");
            if(zoomLoc >= 0)
            {
                glUniform1f(zoomLoc, _zoom);
            }

            GLuint pointRadiutLoc = program->GetUniformLocation("pointRadius");
            if(pointRadiutLoc >= 0)
            {
                glUniform1f(pointRadiutLoc, _pointRadius);
            }

            GLuint halfLineWidthLoc = program->GetUniformLocation("halfLineWidth");
            if(halfLineWidthLoc >= 0)
            {
                glUniform1f(halfLineWidthLoc, _lineWidth / 2.f);
            }


            if(pass == 0)
            {
                renderTriangles(program->GetShaderId());
            }
            else
            {
                renderTriangles(program->GetShaderId(), true);
            }
        }
    }
}
