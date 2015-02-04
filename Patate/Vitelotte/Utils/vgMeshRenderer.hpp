/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "vgMeshRenderer.h"


namespace Vitelotte
{


template < class _Mesh >
VGMeshRenderer<_Mesh>::VGMeshRenderer() :
    m_initialized(false),
    m_useVao(true),

    m_verticesBuffer(0),
    m_indicesBuffer(0),
    m_nodesBuffer(0),
    m_nodesTexture(0),

    m_vao(0)
{}


template < class _Mesh >
VGMeshRenderer<_Mesh>::~VGMeshRenderer()
{
    if(m_initialized)
        releaseGLRessources();
}


template < class _Mesh >
bool VGMeshRenderer<_Mesh>::initialize()
{
    bool ok = true;
    ok &= initSolidShader(m_solidLinearShader, m_solidLinearUniforms,
                          VGMeshRendererShaders::frag_linear_glsl);
    ok &= initSolidShader(m_solidQuadraticShader, m_solidQuadraticUniforms,
                          VGMeshRendererShaders::frag_quadratic_glsl);
    ok &= initWireframeShader();
    m_initialized = ok;
    return ok;
}


template < class _Mesh >
void VGMeshRenderer<_Mesh>::releaseGLRessources()
{
    m_initialized = false;

    m_solidLinearShader.destroy();
    m_solidQuadraticShader.destroy();
    m_wireframeShader.destroy();

    glDeleteBuffers(1, &m_verticesBuffer);
    m_verticesBuffer = 0;
    glDeleteBuffers(1, &m_indicesBuffer);
    m_indicesBuffer = 0;
    glDeleteBuffers(1, &m_nodesBuffer);
    m_nodesBuffer = 0;

    glDeleteTextures(1, &m_nodesTexture);
    m_nodesTexture = 0;

    glDeleteVertexArrays(1, &m_vao);
    m_vao = 0;
}


template < class _Mesh >
void VGMeshRenderer<_Mesh>::updateBuffers(const Mesh& mesh)
{
    PATATE_ASSERT_NO_GL_ERROR();

    m_vertices.clear();
    m_indices.clear();
    m_nodes.clear();

    if(!mesh.hasToVertexValue())
        return;

    m_quadratic = mesh.hasEdgeValue();

    int nodePerTriangle = m_quadratic? 6: 3;

    // Compute number of singular and normal triangles
    m_nSingulars = mesh.nSingularFaces();
    m_nTriangles = mesh.nFaces() - m_nSingulars;

    // Reserve buffers
    m_vertices.reserve(mesh.nVertices());
    m_indices.resize(m_nTriangles * 3 + m_nSingulars * 3);
    m_nodes.resize(m_nTriangles * nodePerTriangle +
                   m_nSingulars * (nodePerTriangle + 1));

    // Push vertices positions
    for(typename Mesh::VertexIterator vit = mesh.verticesBegin();
        vit != mesh.verticesEnd(); ++vit)
    {
        m_vertices.push_back(mesh.position(*vit));
    }

    // Push faces indices and nodes
    unsigned triIndex = 0;
    unsigned singIndex = m_nTriangles * 3;
    unsigned triNodeIndex = 0;
    unsigned singNodeIndex = m_nTriangles * nodePerTriangle;
    for(typename Mesh::FaceIterator fit = mesh.facesBegin();
        fit != mesh.facesEnd(); ++fit)
    {
        // Ensure we work with triangles
        assert(mesh.valence(*fit) == 3);
        typename Mesh::Halfedge h = mesh.halfedge(*fit);

        bool isSingular = mesh.nSingulars(*fit);
        if(isSingular)
        {
            // The first vertex must be the singular one
            while(!mesh.isSingular(h)) { h = mesh.nextHalfedge(h); }
        }

        unsigned& index = isSingular? singIndex: triIndex;
        unsigned& nodeIndex = isSingular? singNodeIndex: triNodeIndex;
        // Push vertices nodes
        for(int ei = 0; ei < 3; ++ei)
        {
            m_indices[index + ei] = mesh.toVertex(h).idx();
            h = mesh.nextHalfedge(h);
            m_nodes[nodeIndex + ei] = nodeValue(mesh, mesh.fromVertexValueNode(h));
        }
        // Singular node is the last one
        if(isSingular)
            m_nodes[nodeIndex + nodePerTriangle] = nodeValue(mesh, mesh.toVertexValueNode(h));

        if(m_quadratic)
        {
            // Push edge nodes
            h = mesh.prevHalfedge(h);
            for(int ei = 0; ei < 3; ++ei)
            {
                m_nodes[nodeIndex + 3 + ei] = nodeValue(mesh, mesh.edgeValueNode(h));
                h = mesh.nextHalfedge(h);
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
void VGMeshRenderer<_Mesh>::drawGeometry(unsigned geomFlags)
{
    assert((geomFlags & ~ALL_TRIANGLES) == 0);
    PATATE_ASSERT_NO_GL_ERROR();

    unsigned nPrimitives = 0;
    unsigned firstPrimitive = 0;
    if(geomFlags & SINGULAR_TRIANGLES)
    {
        nPrimitives += m_nSingulars;
        firstPrimitive = m_nTriangles;
    }
    if(geomFlags & NORMAL_TRIANGLES)
    {
        nPrimitives += m_nTriangles;
        firstPrimitive = 0;
    }

    if(nPrimitives == 0)
    {
        return;
    }

    bool setupBuffers = !m_useVao || !m_vao;
    if(m_useVao)
    {
        if(!m_vao)
            glGenVertexArrays(1, &m_vao);
        glBindVertexArray(m_vao);
    }

    if(setupBuffers)
    {
        glEnableVertexAttribArray(VG_MESH_POSITION_ATTR_LOC);
        glBindBuffer(GL_ARRAY_BUFFER, m_verticesBuffer);
        glVertexAttribPointer(VG_MESH_POSITION_ATTR_LOC,
                              Vector::SizeAtCompileTime, GL_FLOAT,
                              false, sizeof(Vector), 0);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indicesBuffer);
    }

    glDrawElements(GL_TRIANGLES, nPrimitives * 3, GL_UNSIGNED_INT,
                  (const void*)(firstPrimitive * 3 * sizeof(unsigned)));

    if(m_useVao)
        glBindVertexArray(0);
    else
        glDisableVertexAttribArray(VG_MESH_POSITION_ATTR_LOC);

    PATATE_ASSERT_NO_GL_ERROR();
}


template < class _Mesh >
void VGMeshRenderer<_Mesh>::render(const Eigen::Matrix4f& viewMatrix)
{
    PATATE_ASSERT_NO_GL_ERROR();

    PatateCommon::Shader& shader = m_quadratic?
                m_solidQuadraticShader:
                m_solidLinearShader;

    SolidUniforms& unif = m_quadratic?
                m_solidQuadraticUniforms:
                m_solidLinearUniforms;

    shader.use();

    glUniformMatrix4fv(unif.viewMatrixLoc, 1, false, viewMatrix.data());
    glUniform1i(unif.nodesLoc, NODES_TEXTURE_UNIT);

    glActiveTexture(GL_TEXTURE0 + NODES_TEXTURE_UNIT);
    glBindTexture(GL_TEXTURE_BUFFER, m_nodesTexture);

    // Normal triangles
    glUniform1i(unif.baseNodeIndexLoc, 0);
    glUniform1i(unif.singularTrianglesLoc, false);

    drawGeometry(NORMAL_TRIANGLES);

    // Singular triangles
    int nodePerTriangle = m_quadratic? 6: 3;
    glUniform1i(unif.baseNodeIndexLoc, m_nTriangles * nodePerTriangle);
    glUniform1i(unif.singularTrianglesLoc, true);

    drawGeometry(SINGULAR_TRIANGLES);

    PATATE_ASSERT_NO_GL_ERROR();
}


template < class _Mesh >
void VGMeshRenderer<_Mesh>::renderWireframe(
        const Eigen::Matrix4f& viewMatrix, float zoom, float lineWidth,
        const Eigen::Vector4f& color)
{
    PATATE_ASSERT_NO_GL_ERROR();

    m_wireframeShader.use();
    WireframeUniforms& unif = m_wireframeUniforms;

    glUniformMatrix4fv(unif.viewMatrixLoc, 1, false, viewMatrix.data());
    glUniform1f(unif.zoomLoc, zoom);
    glUniform1f(unif.lineWidthLoc, lineWidth);
    glUniform4fv(unif.wireframeColorLoc, 1, color.data());

    drawGeometry(ALL_TRIANGLES);

    PATATE_ASSERT_NO_GL_ERROR();
}


template < class _Mesh >
bool VGMeshRenderer<_Mesh>::initSolidShader(
        PatateCommon::Shader& shader, SolidUniforms& unif, const char* fragCode)
{
    shader.create();

    bool ok = true;
    ok &= shader.addShader(GL_VERTEX_SHADER,
                           VGMeshRendererShaders::vert_common_glsl);
    ok &= shader.addShader(GL_GEOMETRY_SHADER,
                           VGMeshRendererShaders::geom_common_glsl);
    ok &= shader.addShader(GL_FRAGMENT_SHADER,
                           VGMeshRendererShaders::frag_common_glsl);
    ok &= shader.addShader(GL_FRAGMENT_SHADER, fragCode);

    shader.bindAttributeLocation("vx_position", VG_MESH_POSITION_ATTR_LOC);
    ok &= shader.finalize();

    if(!ok)
        return false;

    unif.viewMatrixLoc        = shader.getUniformLocation("viewMatrix");
    unif.nodesLoc             = shader.getUniformLocation("nodes");
    unif.baseNodeIndexLoc     = shader.getUniformLocation("baseNodeIndex");
    unif.singularTrianglesLoc = shader.getUniformLocation("singularTriangles");

    return true;
}


template < class _Mesh >
bool VGMeshRenderer<_Mesh>::initWireframeShader()
{
    PatateCommon::Shader& shader = m_wireframeShader;
    WireframeUniforms& unif = m_wireframeUniforms;
    shader.create();

    bool ok = true;
    ok &= shader.addShader(GL_VERTEX_SHADER,
                           VGMeshRendererShaders::vert_common_glsl);
    ok &= shader.addShader(GL_GEOMETRY_SHADER,
                           VGMeshRendererShaders::geom_common_glsl);
    ok &= shader.addShader(GL_FRAGMENT_SHADER,
                           VGMeshRendererShaders::frag_common_glsl);
    ok &= shader.addShader(GL_FRAGMENT_SHADER,
                           VGMeshRendererShaders::frag_wireframe_glsl);

    shader.bindAttributeLocation("vx_position", VG_MESH_POSITION_ATTR_LOC);
    ok &= shader.finalize();

    if(!ok)
        return false;

    unif.viewMatrixLoc        = shader.getUniformLocation("viewMatrix");
    unif.zoomLoc              = shader.getUniformLocation("zoom");
    unif.lineWidthLoc         = shader.getUniformLocation("lineWidth");
    unif.wireframeColorLoc    = shader.getUniformLocation("wireframeColor");

    return true;
}


template < class _Mesh >
inline typename VGMeshRenderer<_Mesh>::NodeValue
VGMeshRenderer<_Mesh>::nodeValue(const Mesh& mesh, Node node) const
{
    if(mesh.isValid(node) && mesh.isConstraint(node))
    {
        return PatateCommon::srgbToLinear(mesh.nodeValue(node));
    }
    return NodeValue(0, 0, 0, 1);  // FIXME: Make this class work for Chan != 4
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
