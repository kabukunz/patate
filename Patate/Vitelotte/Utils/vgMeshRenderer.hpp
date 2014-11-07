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


VGMeshRendererDefaultShaders::VGMeshRendererDefaultShaders()
    : m_viewMatrix(Eigen::Matrix4f::Identity()),
      m_showWireframe(false),
      m_zoom(1),
      m_pointRadius(0),
      m_lineWidth(1)
{
}


GLuint VGMeshRendererDefaultShaders::triangleShader()
{
    if(m_triangleShader.status() == Patate::Shader::Uninitialized)
    {
        m_triangleShader.create();

        bool bRes = true;

        bRes &= bRes && m_triangleShader.addShader(GL_VERTEX_SHADER,
                                                   shader::vert_common_glsl);
        bRes &= bRes && m_triangleShader.addShader(GL_GEOMETRY_SHADER,
                                                   shader::geom_common_glsl);
        bRes &= bRes && m_triangleShader.addShader(GL_FRAGMENT_SHADER,
                                                   shader::frag_common_glsl);
        bRes &= bRes && m_triangleShader.addShader(GL_FRAGMENT_SHADER,
                                                   shader::frag_triangle_glsl);

        bRes &= bRes && m_triangleShader.finalize();

        if(bRes)
            getUniforms(m_triangleShader, m_triangleUniforms);
    }

    bool ok = m_triangleShader.status() == Patate::Shader::CompilationSuccessful;
    if(ok)
    {
        m_triangleShader.use();
        setupUniforms(m_triangleUniforms);
    }

    return ok? m_triangleShader.getShaderId(): 0;
}


GLuint VGMeshRendererDefaultShaders::singularShader()
{
    if(m_singularShader.status() == Patate::Shader::Uninitialized)
    {
        m_singularShader.create();

        bool bRes = true;

        bRes &= bRes && m_singularShader.addShader(GL_VERTEX_SHADER,
                                                   shader::vert_common_glsl);
        bRes &= bRes && m_singularShader.addShader(GL_GEOMETRY_SHADER,
                                                   shader::geom_common_glsl);
        bRes &= bRes && m_singularShader.addShader(GL_FRAGMENT_SHADER,
                                                   shader::frag_common_glsl);
        bRes &= bRes && m_singularShader.addShader(GL_FRAGMENT_SHADER,
                                                   shader::frag_singular_glsl);

        bRes &= bRes && m_singularShader.finalize();

        if(bRes)
            getUniforms(m_singularShader, m_triangleUniforms);
    }

    bool ok = m_singularShader.status() == Patate::Shader::CompilationSuccessful;
    if(ok)
    {
        m_singularShader.use();
        setupUniforms(m_triangleUniforms);
    }

    return ok? m_singularShader.getShaderId(): 0;
}


void
VGMeshRendererDefaultShaders::
    getUniforms(Patate::Shader& shader, Uniforms& uniforms)
{
    uniforms.viewMatrixLoc =
            shader.getUniformLocation("viewMatrix");
    uniforms.showWireframeLoc =
            shader.getUniformLocation("showWireframe");
    uniforms.zoomLoc =
            shader.getUniformLocation("zoom");
    uniforms.pointRadiusLoc =
            shader.getUniformLocation("pointRadius");
    uniforms.halfLineWidthLoc =
            shader.getUniformLocation("halfLineWidth");
    uniforms.wireframeColorLoc =
            shader.getUniformLocation("wireframeColor");
    uniforms.pointColorLoc =
            shader.getUniformLocation("pointColor");
}

void
VGMeshRendererDefaultShaders::
    setupUniforms(const Uniforms& uniforms)
{
    if(uniforms.viewMatrixLoc >= 0)
        glUniformMatrix4fv(uniforms.viewMatrixLoc, 1, false, m_viewMatrix.data());

    if(uniforms.showWireframeLoc >= 0)
        glUniform1i(uniforms.showWireframeLoc, m_showWireframe);

    if(uniforms.zoomLoc >= 0)
        glUniform1f(uniforms.zoomLoc, m_zoom);

    if(uniforms.pointRadiusLoc >= 0)
        glUniform1f(uniforms.pointRadiusLoc, m_pointRadius);

    if(uniforms.halfLineWidthLoc >= 0)
        glUniform1f(uniforms.halfLineWidthLoc, m_lineWidth / 2.f);

    if(uniforms.wireframeColorLoc >= 0)
        glUniform4fv(uniforms.wireframeColorLoc, 1, m_wireframeColor.data());

    if(uniforms.pointColorLoc >= 0)
        glUniform4fv(uniforms.pointColorLoc, 1, m_pointColor.data());
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

    m_vertices.clear();
    m_triangleIndices.clear();
    m_singularIndices.clear();
    m_triangleNodes.clear();
    m_singularNodes.clear();

    if(!m_pMesh) return;
    assert(m_pMesh->getAttributes() & Mesh::Quadratic == Mesh::Quadratic);

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
                m_singularNodes.push_back(nodeValue(m_pMesh->vertexFromValueNode(h)));
            }
            m_singularNodes.push_back(nodeValue(m_pMesh->vertexValueNode(h)));

            // Push edge nodes
            h = m_pMesh->prevHalfedge(h);
            for(int ei = 0; ei < 3; ++ei)
            {
                m_singularNodes.push_back(nodeValue(m_pMesh->edgeValueNode(h)));
                h = m_pMesh->nextHalfedge(h);
            }
        }
        else
        {
            // Push vertices nodes
            for(int ei = 0; ei < 3; ++ei)
            {
                m_triangleIndices.push_back(m_pMesh->toVertex(h).idx());
                m_triangleNodes.push_back(nodeValue(m_pMesh->vertexValueNode(h)));
                h = m_pMesh->nextHalfedge(h);
            }

            // Push edge nodes
            h = m_pMesh->prevHalfedge(h);
            for(int ei = 0; ei < 3; ++ei)
            {
                m_triangleNodes.push_back(nodeValue(m_pMesh->edgeValueNode(h)));
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
inline void VGMeshRenderer<_Mesh>::renderTriangles(GLuint _shader, bool _singular)
{
    const IndicesVector& indices = _singular?
                m_singularIndices: m_triangleIndices;

    if(indices.size() == 0)
    {
        return;
    }

    glBindVertexArray(m_vao);

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
inline typename VGMeshRenderer<_Mesh>::NodeValue
VGMeshRenderer<_Mesh>::nodeValue(Node node) const
{
    if(m_pMesh->isValid(node) && m_pMesh->isConstraint(node))
        return srgbToLinear(m_pMesh->nodeValue(node));
    return NodeValue(0, 0, 0, 1);  // FIXME: Make this class work for Chan != 4
}

template < class _Mesh >
inline void VGMeshRenderer<_Mesh>::render(VGMeshRendererShaders &shaders)
{
    GLuint shaderIds[2] =
    {
        shaders.triangleShader(),
        shaders.singularShader()
    };

    for(int pass = 0; pass < 2; ++pass)
    {
        if(shaderIds[pass])
            renderTriangles(shaderIds[pass], pass);
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
