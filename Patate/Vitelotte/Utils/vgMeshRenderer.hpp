#include "vgMeshRenderer.h"


namespace Vitelotte
{


Eigen::Vector4f linearToSrgb(const Eigen::Vector4f& linear)
{
    Eigen::Vector4f srgb = linear;
    for(int i=0; i<3; ++i)
        srgb(i) = linear(i) > 0.0031308?
                    1.055 * std::pow(linear(i), 1/2.4):
                    12.92 * linear(i);
    return srgb;
}


Eigen::Vector4f srgbToLinear(const Eigen::Vector4f& srgb)
{
    Eigen::Vector4f linear = srgb;
    for(int i=0; i<3; ++i)
        linear(i) = linear(i) > 0.04045?
                    std::pow((linear(i)+0.055) / 1.055, 2.4):
                    linear(i) / 12.92;
    return linear;
}


VGMeshRendererDefaultShader::VGMeshRendererDefaultShader()
    : m_viewMatrix(Eigen::Matrix4f::Identity()),
      m_singularTriangles(false),
      m_nodes(0),
      m_baseNodeIndex(0)
{
}


inline
GLint
VGMeshRendererDefaultShader::verticesAttibLocation(TriangleType triangleType)
{
    return (triangleType & Quadratic)? m_verticesLocQuadratic:
                                       m_verticesLocLinear;
}


inline
bool
VGMeshRendererDefaultShader::useShader(TriangleType triangleType)
{
    bool quadratic = triangleType & Quadratic;
    PatateCommon::Shader& shader = quadratic?
                m_quadraticShader:
                m_linearShader;
    Uniforms& uniforms = quadratic?
                m_quadraticUniforms:
                m_linearUniforms;

    if(shader.status() == PatateCommon::Shader::Uninitialized)
    {
        shader.create();

        bool bRes = true;

        bRes &= shader.addShader(GL_VERTEX_SHADER,
                                   shader::vert_common_glsl);
        bRes &= shader.addShader(GL_GEOMETRY_SHADER,
                                   shader::geom_common_glsl);
        bRes &= shader.addShader(GL_FRAGMENT_SHADER,
                                   shader::frag_common_glsl);
        bRes &= shader.addShader(GL_FRAGMENT_SHADER,
                                   quadratic?
                                     shader::frag_quadratic_glsl:
                                     shader::frag_linear_glsl);

        bRes &= shader.finalize();

        if(bRes)
        {
            getUniforms(shader, uniforms);
            if(quadratic)
            {
                m_verticesLocQuadratic =
                        glGetAttribLocation(shader.getShaderId(), "vx_position");
            }
            else
            {
                m_verticesLocLinear =
                        glGetAttribLocation(shader.getShaderId(), "vx_position");
            }
        }
    }

    bool ok = shader.status() == PatateCommon::Shader::CompilationSuccessful;
    if(ok)
    {
        shader.use();
        m_singularTriangles = (triangleType & Singular);
        setupUniforms(uniforms);
    }

    return ok;
}


inline
void
VGMeshRendererDefaultShader::
    setNodesTexture(TriangleType triangleType, int texUnit, int baseOffset)
{
    m_nodes = texUnit;
    m_baseNodeIndex = baseOffset;
}


inline
void
VGMeshRendererDefaultShader::
    getUniforms(PatateCommon::Shader& shader, Uniforms& uniforms)
{
    uniforms.viewMatrixLoc        = shader.getUniformLocation("viewMatrix");
    uniforms.nodesLoc             = shader.getUniformLocation("nodes");
    uniforms.baseNodeIndexLoc     = shader.getUniformLocation("baseNodeIndex");
    uniforms.singularTrianglesLoc = shader.getUniformLocation("singularTriangles");
}


inline
void
VGMeshRendererDefaultShader::
    setupUniforms(const Uniforms& uniforms)
{
    if(uniforms.viewMatrixLoc >= 0)
        glUniformMatrix4fv(uniforms.viewMatrixLoc, 1, false, m_viewMatrix.data());

    if(uniforms.nodesLoc >= 0)
        glUniform1i(uniforms.nodesLoc, m_nodes);

    if(uniforms.baseNodeIndexLoc >= 0)
        glUniform1i(uniforms.baseNodeIndexLoc, m_baseNodeIndex);

    if(uniforms.singularTrianglesLoc >= 0)
        glUniform1i(uniforms.singularTrianglesLoc, m_singularTriangles);
}



VGMeshRendererWireframeShader::VGMeshRendererWireframeShader()
    : m_viewMatrix(Eigen::Matrix4f::Identity()),
      m_zoom(1.),
      m_lineWidth(1.),
      m_wireframeColor(0., 0., 0., 1.)
{
}


inline
GLint
VGMeshRendererWireframeShader::verticesAttibLocation(TriangleType /*triangleType*/)
{
    return m_verticesLoc;
}


inline
bool
VGMeshRendererWireframeShader::useShader(TriangleType /*triangleType*/)
{
    if(m_shader.status() == PatateCommon::Shader::Uninitialized)
    {
        m_shader.create();

        bool bRes = true;

        bRes &= m_shader.addShader(GL_VERTEX_SHADER,
                                   shader::vert_common_glsl);
        bRes &= m_shader.addShader(GL_GEOMETRY_SHADER,
                                   shader::geom_common_glsl);
        bRes &= m_shader.addShader(GL_FRAGMENT_SHADER,
                                   shader::frag_common_glsl);
        bRes &= m_shader.addShader(GL_FRAGMENT_SHADER,
                                   shader::frag_wireframe_glsl);

        bRes &= m_shader.finalize();

        if(bRes)
        {
            getUniforms();
            m_verticesLoc = glGetAttribLocation(m_shader.getShaderId(), "vx_position");
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
VGMeshRendererWireframeShader::
    setNodesTexture(TriangleType /*triangleType*/, int /*texUnit*/, int /*baseOffset*/)
{
}


inline
void
VGMeshRendererWireframeShader::
    getUniforms()
{
    m_viewMatrixLoc        = m_shader.getUniformLocation("viewMatrix");
    m_zoomLoc              = m_shader.getUniformLocation("zoom");
    m_lineWidthLoc         = m_shader.getUniformLocation("lineWidth");
    m_wireframeColorLoc    = m_shader.getUniformLocation("wireframeColor");
}


inline
void
VGMeshRendererWireframeShader::
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
inline void VGMeshRenderer<_Mesh>::initialize(Mesh* _mesh)
{
    glGenVertexArrays(1, &m_vao);

    setMesh(_mesh);
}


template < class _Mesh >
inline void VGMeshRenderer<_Mesh>::updateMesh()
{
    PATATE_ASSERT_NO_GL_ERROR();

    m_vertices.clear();
    m_indices.clear();
    m_nodes.clear();

    if(!m_pMesh) return;
    assert(m_pMesh->getAttributes() & Mesh::VertexValue);
    m_quadratic = m_pMesh->getAttributes() & Mesh::EdgeValue;

    int nodePerTriangle = m_quadratic? 6: 3;

    // Compute number of singular and normal triangles
    m_nSingulars = m_pMesh->nSingularFaces();
    m_nTriangles = m_pMesh->nFaces() - m_nSingulars;

    // Reserve buffers
    m_vertices.reserve(m_pMesh->nVertices());
    m_indices.resize(m_nTriangles * 3 + m_nSingulars * 3);
    m_nodes.resize(m_nTriangles * nodePerTriangle +
                   m_nSingulars * (nodePerTriangle + 1));

    // Push vertices positions
    for(typename Mesh::VertexIterator vit = m_pMesh->verticesBegin();
        vit != m_pMesh->verticesEnd(); ++vit)
    {
        m_vertices.push_back(m_pMesh->position(*vit));
    }

    // Push faces indices and nodes
    unsigned triIndex = 0;
    unsigned singIndex = m_nTriangles * 3;
    unsigned triNodeIndex = 0;
    unsigned singNodeIndex = m_nTriangles * nodePerTriangle;
    for(typename Mesh::FaceIterator fit = m_pMesh->facesBegin();
        fit != m_pMesh->facesEnd(); ++fit)
    {
        // Ensure we work with triangles
        assert(m_pMesh->valence(*fit) == 3);
        typename Mesh::Halfedge h = m_pMesh->halfedge(*fit);

        bool isSingular = m_pMesh->isSingular(*fit);
        if(isSingular)
        {
            // The first vertex must be the singular one
            while(!m_pMesh->isSingular(h)) { h = m_pMesh->nextHalfedge(h); }
        }

        unsigned& index = isSingular? singIndex: triIndex;
        unsigned& nodeIndex = isSingular? singNodeIndex: triNodeIndex;
        // Push vertices nodes
        for(int ei = 0; ei < 3; ++ei)
        {
            m_indices[index + ei] = m_pMesh->toVertex(h).idx();
            h = m_pMesh->nextHalfedge(h);
            m_nodes[nodeIndex + ei] = nodeValue(m_pMesh->vertexFromValueNode(h));
        }
        // Singular node is the last one
        if(isSingular)
            m_nodes[nodeIndex + nodePerTriangle] = nodeValue(m_pMesh->vertexValueNode(h));

        if(m_quadratic)
        {
            // Push edge nodes
            h = m_pMesh->prevHalfedge(h);
            for(int ei = 0; ei < 3; ++ei)
            {
                m_nodes[nodeIndex + 3 + ei] = nodeValue(m_pMesh->edgeValueNode(h));
                h = m_pMesh->nextHalfedge(h);
            }
        }

        index += 3;
        nodeIndex += nodePerTriangle + isSingular;
    }
    assert(triIndex == m_nTriangles * 3 && singIndex == m_indices.size());
    assert(triNodeIndex == m_nTriangles * nodePerTriangle && singNodeIndex == m_nodes.size());

    // Create and upload buffers
    createAndUploadBuffer(m_verticesBuffer, GL_ARRAY_BUFFER,
                          m_vertices);

    createAndUploadBuffer(m_indicesBuffer, GL_ELEMENT_ARRAY_BUFFER,
                          m_indices);

    createAndUploadBuffer(m_nodesBuffer, GL_ARRAY_BUFFER,
                          m_nodes);

    // Create and setup texture buffer
    if(!m_nodesTexture)
    {
        glGenTextures(1, &m_nodesTexture);
    }
    glBindTexture(GL_TEXTURE_BUFFER, m_nodesTexture);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, m_nodesBuffer);

    PATATE_ASSERT_NO_GL_ERROR();
}


template < class _Mesh >
inline void VGMeshRenderer<_Mesh>::renderTriangles(
        VGMeshRendererShader& shaders, bool _singular)
{
    PATATE_ASSERT_NO_GL_ERROR();

    unsigned nPrimitives = (!_singular)?
                m_nTriangles: m_nSingulars;

    if(nPrimitives == 0)
    {
        return;
    }

    glBindVertexArray(m_vao);

    VGMeshRendererShader::TriangleType triangleType =
            VGMeshRendererShader::TriangleType(
                (m_quadratic? VGMeshRendererShader::Quadratic: 0) |
                (_singular?   VGMeshRendererShader::Singular:  0));

    int nodePerTriangle = m_quadratic? 6: 3;
    shaders.setNodesTexture(triangleType, 0, _singular * m_nTriangles * nodePerTriangle);
    if(!shaders.useShader(triangleType))
        return;

    GLint verticesLoc = shaders.verticesAttibLocation(triangleType);
    if(verticesLoc >= 0)
    {
        glEnableVertexAttribArray(verticesLoc);
        glBindBuffer(GL_ARRAY_BUFFER, m_verticesBuffer);
        glVertexAttribPointer(verticesLoc, Vector::SizeAtCompileTime, GL_FLOAT,
                              false, sizeof(Vector), 0);
    }

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_BUFFER, m_nodesTexture);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indicesBuffer);
    glDrawElements(GL_TRIANGLES, nPrimitives * 3, GL_UNSIGNED_INT,
                  (const void*)(_singular * m_nTriangles * 3 * sizeof(unsigned)));

    if(verticesLoc >= 0)
    {
        glDisableVertexAttribArray(verticesLoc);
    }

    PATATE_ASSERT_NO_GL_ERROR();
}


template < class _Mesh >
inline typename VGMeshRenderer<_Mesh>::NodeValue
VGMeshRenderer<_Mesh>::nodeValue(Node node) const
{
    if(m_pMesh->isValid(node) && m_pMesh->isConstraint(node))
        return srgbToLinear(m_pMesh->nodeValue(node));
    return NodeValue(0, 0, 0, 1);  // FIXME: Make this class work for Chan != 4
}


template < class _Mesh >
inline void VGMeshRenderer<_Mesh>::render(VGMeshRendererShader &shaders)
{
    for(int pass = 0; pass < 2; ++pass)
    {
        renderTriangles(shaders, pass);
    }
}


template < class _Mesh >
template < typename T >
void VGMeshRenderer<_Mesh>::createAndUploadBuffer(
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
