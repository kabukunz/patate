

template < class _Mesh >
inline bool QMeshRenderer<_Mesh>::init(Mesh* _mesh)
{
    if(!loadShaders())
    {
        return false;
    }

    if(!initGl())
        return false;

    setMesh(_mesh);

    return true;
}

template < class _Mesh >
inline void QMeshRenderer<_Mesh>::updateMesh()
{
    m_vertices.clear();
    m_triangleIndices.clear();
    m_singularIndices.clear();
    m_triangleNodes.clear();
    m_singularNodes.clear();

    if(!m_pMesh) return;

    // Compute number of singular and normal triangles
    unsigned nSingulars = m_pMesh->nSingularFaces();
    unsigned nTriangles = m_pMesh->nFaces() - nSingulars;

    // Reserve buffers
    m_vertices.reserve(m_pMesh->nVertices());
    m_triangleIndices.reserve(nTriangles * 3);
    m_singularIndices.reserve(nSingulars * 3);
    m_triangleNodes.reserve(nTriangles * 6);
    m_singularNodes.reserve(nSingulars * 7);

    // Push vertices positions
    for(typename Mesh::VertexIterator vit = m_pMesh->verticesBegin();
        vit != m_pMesh->verticesEnd(); ++vit)
    {
        m_vertices.push_back(m_pMesh->position(*vit));
    }

    // Push faces indices and nodes
    for(typename Mesh::FaceIterator fit = m_pMesh->facesBegin();
        fit != m_pMesh->facesEnd(); ++fit)
    {
        // Ensure we work with triangles
        assert(m_pMesh->valence(*fit) == 3);
        typename Mesh::Halfedge h = m_pMesh->halfedge(*fit);

        if(m_pMesh->isSingular(*fit))
        {
            // The first vertex must be the singular one
            while(!m_pMesh->isSingular(h)) { h = m_pMesh->nextHalfedge(h); }

            // Push vertices nodes
            for(int ei = 0; ei < 3; ++ei)
            {
                m_singularIndices.push_back(m_pMesh->toVertex(h).idx());
                h = m_pMesh->nextHalfedge(h);
                m_singularNodes.push_back(nodeValue(m_pMesh->fromNode(h)));
            }
            m_singularNodes.push_back(nodeValue(m_pMesh->toNode(h)));

            // Push edge nodes
            h = m_pMesh->prevHalfedge(h);
            for(int ei = 0; ei < 3; ++ei)
            {
                m_singularNodes.push_back(nodeValue(m_pMesh->midNode(h)));
                h = m_pMesh->nextHalfedge(h);
            }
        }
        else
        {
            // Push vertices nodes
            for(int ei = 0; ei < 3; ++ei)
            {
                m_triangleIndices.push_back(m_pMesh->toVertex(h).idx());
                m_triangleNodes.push_back(nodeValue(m_pMesh->toNode(h)));
                h = m_pMesh->nextHalfedge(h);
            }

            // Push edge nodes
            h = m_pMesh->prevHalfedge(h);
            for(int ei = 0; ei < 3; ++ei)
            {
                m_triangleNodes.push_back(nodeValue(m_pMesh->midNode(h)));
                h = m_pMesh->nextHalfedge(h);
            }
        }
    }

    // Create and upload buffers
    createAndUploadBuffer(m_verticesBuffer, GL_ARRAY_BUFFER,
                          m_vertices);

    createAndUploadBuffer(m_triangleIndicesBuffer, GL_ELEMENT_ARRAY_BUFFER,
                          m_triangleIndices);
    createAndUploadBuffer(m_triangleNodesBuffer, GL_ARRAY_BUFFER,
                          m_triangleNodes);

    createAndUploadBuffer(m_singularIndicesBuffer, GL_ELEMENT_ARRAY_BUFFER,
                          m_singularIndices);
    createAndUploadBuffer(m_singularNodesBuffer, GL_ARRAY_BUFFER,
                          m_singularNodes);

    // Create and setup texture buffers
    if(!m_triangleNodesTexture)
    {
        glGenTextures(1, &m_triangleNodesTexture);
    }
    glBindTexture(GL_TEXTURE_BUFFER, m_triangleNodesTexture);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, m_triangleNodesBuffer);

    if(!m_singularNodesTexture)
    {
        glGenTextures(1, &m_singularNodesTexture);
    }
    glBindTexture(GL_TEXTURE_BUFFER, m_singularNodesTexture);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, m_singularNodesBuffer);
}

template < class _Mesh >
inline bool QMeshRenderer<_Mesh>::loadShaders()
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

template < class _Mesh >
inline bool QMeshRenderer<_Mesh>::initGl()
{
    glGenVertexArrays(1, &m_vao);
    glBindVertexArray(m_vao);
    return true;
}

template < class _Mesh >
inline void QMeshRenderer<_Mesh>::renderTriangles(GLuint _shader, bool _singular)
{
    const IndicesVector& indices = _singular?
                m_singularIndices: m_triangleIndices;

    if(indices.size() == 0)
    {
        return;
    }

    glUseProgram(_shader);

    GLint verticesLoc = glGetAttribLocation(_shader, "vx_position");

    if(verticesLoc >= 0)
    {
        glEnableVertexAttribArray(verticesLoc);
        glBindBuffer(GL_ARRAY_BUFFER, m_verticesBuffer);
        glVertexAttribPointer(verticesLoc, Vector::SizeAtCompileTime, GL_FLOAT,
                              false, sizeof(Vector), 0);
    }

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_BUFFER,
                  _singular ? m_singularNodesTexture : m_triangleNodesTexture);
    glUniform1i(glGetUniformLocation(_shader, "nodes"), 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,
                 _singular ? m_singularIndicesBuffer : m_triangleIndicesBuffer);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

    if(verticesLoc >= 0)
    {
        glDisableVertexAttribArray(verticesLoc);
    }
}

template < class _Mesh >
inline typename QMeshRenderer<_Mesh>::NodeValue
QMeshRenderer<_Mesh>::nodeValue(NodeID node) const
{
    if(m_pMesh->isValid(node) && m_pMesh->isConstraint(node))
        return m_pMesh->nodeValue(node);
    return NodeValue(0, 0, 0, 1);  // FIXME: Make this class work for Chan != 4
}

template < class _Mesh >
inline void QMeshRenderer<_Mesh>::render(Eigen::Matrix4f& _viewMatrix, float _zoom, float _pointRadius, float _lineWidth, bool _showShaderWireframe)
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

template < class _Mesh >
template < typename T >
void QMeshRenderer<_Mesh>::createAndUploadBuffer(
        GLuint& glId, GLenum type, const std::vector<T>& data, GLenum usage)
{
    if(!glId)
    {
        glGenBuffers(1, &glId);
    }
    glBindBuffer(type, glId);
    glBufferData(type, data.size() * sizeof(T),
                 &(data[0]), usage);
}

