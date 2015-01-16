/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "vgMeshGL2Renderer.h"


namespace Vitelotte
{


VGMeshRendererGL2DefaultShader::VGMeshRendererGL2DefaultShader()
    : m_nodesTextureSizeLoc(0),
      m_viewMatrix(Eigen::Matrix4f::Identity()),
      m_nodes(0),
      m_singularTriangles(false)
{
}


inline
bool
VGMeshRendererGL2DefaultShader::useShader(TriangleType triangleType)
{
    if(m_shader.status() == PatateCommon::Shader::Uninitialized)
    {
        m_shader.create();
#ifdef VITELOTTE_USE_OPENGL_ES
        m_shader.setGLSLVersionHeader("#version 100\n");
#else
        m_shader.setGLSLVersionHeader("#version 120\n");
#endif

        bool bRes = true;

        bRes &= m_shader.addShader(GL_VERTEX_SHADER,
                                   VGMeshGL2RendererShaders::vert_common_glsl);
        bRes &= m_shader.addShader(GL_FRAGMENT_SHADER,
                                   VGMeshGL2RendererShaders::frag_quadratic_glsl);

        bRes &= m_shader.finalize();

        if(bRes)
        {
            getUniforms();
            m_verticesLoc = glGetAttribLocation(m_shader.getShaderId(), "vx_position");
            m_basisLoc = glGetAttribLocation(m_shader.getShaderId(), "vx_basis");
            m_edgesLoc = glGetAttribLocation(m_shader.getShaderId(), "vx_edges");
            m_baseNodeCoordLoc = glGetAttribLocation(m_shader.getShaderId(), "vx_baseNodeCoord");
        }
    }

    bool ok = m_shader.status() == PatateCommon::Shader::CompilationSuccessful;
    if(ok)
    {
        m_shader.use();
        m_singularTriangles = (triangleType == Singular);
        setupUniforms();
    }

    return ok;
}


inline
void
VGMeshRendererGL2DefaultShader::
    setNodesTexture(int texUnit, int size)
{
    m_nodes = texUnit;
    m_textureSize = size;
}


inline
void
VGMeshRendererGL2DefaultShader::
    getUniforms()
{
    m_viewMatrixLoc        = m_shader.getUniformLocation("viewMatrix");
    m_nodesLoc             = m_shader.getUniformLocation("nodes");
    m_singularTrianglesLoc = m_shader.getUniformLocation("singularTriangles");
    m_nodesTextureSizeLoc  = m_shader.getUniformLocation("nodesTextureSize");
}


inline
void
VGMeshRendererGL2DefaultShader::
    setupUniforms()
{
    if(m_viewMatrixLoc >= 0)
        glUniformMatrix4fv(m_viewMatrixLoc, 1, false, m_viewMatrix.data());

    if(m_nodesLoc >= 0)
        glUniform1i(m_nodesLoc, m_nodes);

    if(m_singularTrianglesLoc >= 0)
        glUniform1i(m_singularTrianglesLoc, m_singularTriangles);

    if(m_nodesTextureSizeLoc >= 0)
        glUniform1i(m_nodesTextureSizeLoc, m_textureSize);
}



VGMeshRendererGL2WireframeShader::VGMeshRendererGL2WireframeShader()
    : m_viewMatrix(Eigen::Matrix4f::Identity()),
      m_zoom(1.),
      m_lineWidth(1.),
      m_wireframeColor(0., 0., 0., 1.)
{
}


inline
bool
VGMeshRendererGL2WireframeShader::useShader(TriangleType /*triangleType*/)
{
    if(m_shader.status() == PatateCommon::Shader::Uninitialized)
    {
        m_shader.create();
#ifdef VITELOTTE_USE_OPENGL_ES
        m_shader.setGLSLVersionHeader("#version 100\n");
#else
        m_shader.setGLSLVersionHeader("#version 120\n");
#endif

        bool bRes = true;

        bRes &= m_shader.addShader(GL_VERTEX_SHADER,
                                   VGMeshGL2RendererShaders::vert_common_glsl);
        bRes &= m_shader.addShader(GL_FRAGMENT_SHADER,
                                   VGMeshGL2RendererShaders::frag_wireframe_glsl);

        bRes &= m_shader.finalize();

        if(bRes)
        {
            getUniforms();
            m_verticesLoc = glGetAttribLocation(m_shader.getShaderId(), "vx_position");
            m_basisLoc = glGetAttribLocation(m_shader.getShaderId(), "vx_basis");
            m_edgesLoc = glGetAttribLocation(m_shader.getShaderId(), "vx_edges");
        }
    }

    bool ok = m_shader.status() == PatateCommon::Shader::CompilationSuccessful;
    if(ok)
    {
        m_shader.use();
        setupUniforms();
    }

    return ok;
}


inline
void
VGMeshRendererGL2WireframeShader::
    setNodesTexture(int /*texUnit*/, int /*size*/)
{
}


inline
void
VGMeshRendererGL2WireframeShader::
    getUniforms()
{
    m_viewMatrixLoc        = m_shader.getUniformLocation("viewMatrix");
    m_zoomLoc              = m_shader.getUniformLocation("zoom");
    m_lineWidthLoc         = m_shader.getUniformLocation("lineWidth");
    m_wireframeColorLoc    = m_shader.getUniformLocation("wireframeColor");
}


inline
void
VGMeshRendererGL2WireframeShader::
    setupUniforms()
{
    if(m_viewMatrixLoc >= 0)
        glUniformMatrix4fv(m_viewMatrixLoc, 1, false, m_viewMatrix.data());

    if(m_lineWidthLoc >= 0)
        glUniform1f(m_lineWidthLoc, m_lineWidth);

    if(m_zoomLoc >= 0)
        glUniform1f(m_zoomLoc, m_zoom);

    if(m_wireframeColorLoc >= 0)
        glUniform4fv(m_wireframeColorLoc, 1, m_wireframeColor.data());
}


template < class _Mesh >
inline void VGMeshGL2Renderer<_Mesh>::initialize(Mesh* _mesh)
{
//    glGenVertexArrays(1, &m_vao);

    setMesh(_mesh);
}


template < class _Mesh >
inline void VGMeshGL2Renderer<_Mesh>::updateMesh()
{
    PATATE_ASSERT_NO_GL_ERROR();

    m_vertices.clear();
    m_nodes.clear();

    if(!m_pMesh) return;
    assert((m_pMesh->getAttributes() & Mesh::QUADRATIC_FLAGS) == Mesh::QUADRATIC_FLAGS);

    // Compute number of singular and normal triangles
    m_nSingulars = m_pMesh->nSingularFaces();
    m_nTriangles = m_pMesh->nFaces() - m_nSingulars;
    //int nNodes = m_nTriangles * 6 + m_nSingulars * 7;
    m_nodeTextureSize = 4;
    int nTriRows = 0;
    int nSingRows = 0;
    do
    {
        m_nodeTextureSize *= 2;
        int nTriPerCol = m_nodeTextureSize / 6;
        int nSingPerCol = m_nodeTextureSize / 7;
        nTriRows = (m_nTriangles + (nTriPerCol - 1)) / nTriPerCol;
        nSingRows = (m_nSingulars + (nSingPerCol - 1)) / nSingPerCol;
    } while(m_nodeTextureSize < nTriRows + nSingRows);


    // Reserve buffers
    m_vertices.resize(m_pMesh->nFaces() * 3);
    m_nodes.resize(m_nodeTextureSize*m_nodeTextureSize);

    // Push faces indices and nodes
    unsigned triIndex = 0;
    unsigned singIndex = m_nTriangles * 3;
    Eigen::Vector2i triNodeCoord(0, 0);
    Eigen::Vector2i singNodeCoord(0, nTriRows);
    for(typename Mesh::FaceIterator fit = m_pMesh->facesBegin();
        fit != m_pMesh->facesEnd(); ++fit)
    {
        // Ensure we work with triangles
        assert(m_pMesh->valence(*fit) == 3);
        typename Mesh::Halfedge h = m_pMesh->halfedge(*fit);

        bool isSingular = m_pMesh->nSingulars(*fit);
        if(isSingular)
        {
            // The first vertex must be the singular one
            while(!m_pMesh->isSingular(h)) { h = m_pMesh->nextHalfedge(h); }
        }

        Vector vec[3];
        h = m_pMesh->prevHalfedge(h);
        for(int ei = 0; ei < 3; ++ei)
        {
            vec[ei] = m_pMesh->position(m_pMesh->toVertex(h))
                    - m_pMesh->position(m_pMesh->fromVertex(h));
            h = m_pMesh->nextHalfedge(h);
        }
        h = m_pMesh->nextHalfedge(h);

        unsigned& index = isSingular? singIndex: triIndex;
        Eigen::Vector2i& nodeCoord = isSingular? singNodeCoord: triNodeCoord;
        int nodeIndex = nodeCoord.x() + nodeCoord.y() * m_nodeTextureSize;
        // Push vertices nodes
        for(int ei = 0; ei < 3; ++ei)
        {
            Vertex vert;
            vert.pos = m_pMesh->position(m_pMesh->toVertex(h));
            vert.basis = 1 << (8*ei);
            vert.edge0 = vec[0];
            vert.edge1 = vec[1];
            vert.baseNodeCoord =
                    (nodeCoord.cast<float>() + Eigen::Vector2f(.5, .5)) /
                            float(m_nodeTextureSize);
            m_vertices[index + ei] = vert;
            m_nodes[nodeIndex + 3 + ((ei+1)%3)] = nodeValue(m_pMesh->edgeValueNode(h));

            h = m_pMesh->nextHalfedge(h);

            m_nodes[nodeIndex + ei] = nodeValue(m_pMesh->fromVertexValueNode(h));
        }
        // Singular node is the last one
        if(isSingular)
        {
            m_nodes[nodeIndex + 6] = nodeValue(m_pMesh->toVertexValueNode(h));
        }

        index += 3;
        nodeCoord.x() += 6 + isSingular;
        if(nodeCoord.x() + 6 + isSingular >= m_nodeTextureSize)
            nodeCoord = Eigen::Vector2i(0, nodeCoord.y() + 1);
    }
    assert(triIndex == m_nTriangles * 3);
    assert(singIndex == m_vertices.size());
    //assert(triNodeCoord.y() == nTriRows-1 && singNodeCoord.y() == nSingRows-1);

    // Create and upload buffers
    createAndUploadBuffer(m_verticesBuffer, GL_ARRAY_BUFFER,
                          m_vertices);

    // Create and setup texture
    if(!m_nodesTexture)
    {
        glGenTextures(1, &m_nodesTexture);
    }
    glBindTexture(GL_TEXTURE_2D, m_nodesTexture);
    glTexImage2D(GL_TEXTURE_2D, 0,
#ifdef VITELOTTE_USE_OPENGL_ES
                 GL_RGBA,
#else
                 GL_SRGB8_ALPHA8,
#endif
                 m_nodeTextureSize, m_nodeTextureSize, 0,
                 GL_RGBA, GL_FLOAT, &m_nodes[0]);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    PATATE_ASSERT_NO_GL_ERROR();
}


template < class _Mesh >
inline void VGMeshGL2Renderer<_Mesh>::renderTriangles(
        VGMeshGL2RendererShader& shaders, bool _singular)
{
    PATATE_ASSERT_NO_GL_ERROR();

    unsigned nPrimitives = (!_singular)?
                m_nTriangles: m_nSingulars;

    if(nPrimitives == 0)
    {
        return;
    }

    // MUST be called before useShader !
    shaders.setNodesTexture(0, m_nodeTextureSize);
    if(!shaders.useShader((!_singular)?
                          VGMeshGL2RendererShader::Standard:
                          VGMeshGL2RendererShader::Singular))
        return;

//    glBindVertexArray(m_vao);

    glBindBuffer(GL_ARRAY_BUFFER, m_verticesBuffer);

    PATATE_ASSERT_NO_GL_ERROR();
    GLint verticesLoc = shaders.verticesAttibLocation();
    if(verticesLoc >= 0)
    {
        glEnableVertexAttribArray(verticesLoc);
        glVertexAttribPointer(verticesLoc, Vector::SizeAtCompileTime, GL_FLOAT,
                              false, sizeof(Vertex), 0);
    }

    PATATE_ASSERT_NO_GL_ERROR();
    GLint edgesLoc = shaders.edgesAttibLocation();
    if(edgesLoc >= 0)
    {
        glEnableVertexAttribArray(edgesLoc);
        glVertexAttribPointer(edgesLoc, 4, GL_FLOAT,
                              false, sizeof(Vertex),
                              PATATE_FIELD_OFFSET(Vertex, edge0));
    }

    PATATE_ASSERT_NO_GL_ERROR();
    GLint baseNodeCoordLoc = shaders.baseNodeCoordAttibLocation();
    if(baseNodeCoordLoc >= 0)
    {
        glEnableVertexAttribArray(baseNodeCoordLoc);
        glVertexAttribPointer(baseNodeCoordLoc, 2, GL_FLOAT,
                              false, sizeof(Vertex),
                              PATATE_FIELD_OFFSET(Vertex, baseNodeCoord));
    }

    PATATE_ASSERT_NO_GL_ERROR();
    GLint basisLoc = shaders.basisAttibLocation();
    if(basisLoc >= 0)
    {
        glEnableVertexAttribArray(basisLoc);
        glVertexAttribPointer(basisLoc, 3, GL_UNSIGNED_BYTE,
                              false, sizeof(Vertex),
                              PATATE_FIELD_OFFSET(Vertex, basis));
    }

    PATATE_ASSERT_NO_GL_ERROR();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_nodesTexture);

    PATATE_ASSERT_NO_GL_ERROR();
    glDrawArrays(GL_TRIANGLES,
                 _singular * m_nTriangles * 3,
                 nPrimitives * 3);

    if(basisLoc >= 0)
        glDisableVertexAttribArray(basisLoc);
    if(baseNodeCoordLoc >= 0)
        glDisableVertexAttribArray(baseNodeCoordLoc);
    if(edgesLoc >= 0)
        glDisableVertexAttribArray(edgesLoc);
    if(verticesLoc >= 0)
        glDisableVertexAttribArray(verticesLoc);

    PATATE_ASSERT_NO_GL_ERROR();
}


template < class _Mesh >
inline typename VGMeshGL2Renderer<_Mesh>::NodeValue
VGMeshGL2Renderer<_Mesh>::nodeValue(Node node) const
{
    if(m_pMesh->isValid(node) && m_pMesh->isConstraint(node))
    {
        return m_pMesh->nodeValue(node);
    }
    return NodeValue(0, 0, 0, 1);  // FIXME: Make this class work for Chan != 4
}


template < class _Mesh >
inline void VGMeshGL2Renderer<_Mesh>::render(VGMeshGL2RendererShader &shaders)
{
    for(int pass = 0; pass < 2; ++pass)
    {
        renderTriangles(shaders, pass);
    }
}


template < class _Mesh >
template < typename T >
void VGMeshGL2Renderer<_Mesh>::createAndUploadBuffer(
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

}
