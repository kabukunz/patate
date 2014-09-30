#ifndef _QMESH_RENDERER_BASE_H_
#define _QMESH_RENDERER_BASE_H_

#include <GL/glew.h>

#include "qMesh.h"
#include "../../common/GLUtils/technique.h"


static const char* vert_common = "                                                      \n\
#version 410 core                                                                       \n\
                                                                                        \n\
uniform mat4 viewMatrix;                                                                \n\
                                                                                        \n\
in vec4 vx_position;                                                                    \n\
                                                                                        \n\
out vec2 position_obj;                                                                  \n\
                                                                                        \n\
void main(void)                                                                         \n\
{                                                                                       \n\
    gl_Position = viewMatrix * vx_position;                                             \n\
    position_obj = vx_position.xy;                                                      \n\
}";

static const char* geom_common = "                                                      \n\
#version 410 core                                                                       \n\
                                                                                        \n\
layout(triangles) in;                                                                   \n\
layout(triangle_strip, max_vertices = 3) out;                                           \n\
                                                                                        \n\
in vec2 position_obj[];                                                                 \n\
                                                                                        \n\
out int gl_PrimitiveID;                                                                 \n\
out vec3 linearBasis;                                                                   \n\
out vec2 position;                                                                      \n\
flat out vec2 vertices[3];                                                              \n\
flat out vec2 normEdges[3];                                                             \n\
                                                                                        \n\
const vec3 basis[3] = vec3[3](                                                          \n\
    vec3(1, 0, 0),                                                                      \n\
    vec3(0, 1, 0),                                                                      \n\
    vec3(0, 0, 1)                                                                       \n\
    );                                                                                  \n\
                                                                                        \n\
void main()                                                                             \n\
{                                                                                       \n\
    for(int i=0; i<3; ++i)                                                              \n\
    {                                                                                   \n\
        gl_Position = gl_in[i].gl_Position;                                             \n\
        gl_PrimitiveID = gl_PrimitiveIDIn;                                              \n\
        linearBasis = basis[i];                                                         \n\
        position = position_obj[i];//gl_in[i].gl_Position.xy;                           \n\
        for(int j=0; j<3; ++j)                                                          \n\
        {                                                                               \n\
            vertices[j] = position_obj[j];//gl_in[j].gl_Position.xy;                    \n\
            normEdges[j] = normalize(vertices[(j+2)%3] - vertices[(j+1)%3]);            \n\
        }                                                                               \n\
        EmitVertex();                                                                   \n\
    }                                                                                   \n\
}";

static const char* frag_common = "                                                      \n\
#version 410 core                                                                       \n\
                                                                                        \n\
uniform float zoom;                                                                     \n\
uniform float pointRadius;                                                              \n\
uniform float halfLineWidth;                                                            \n\
uniform bool showWireframe;                                                             \n\
                                                                                        \n\
in vec3 linearBasis;                                                                    \n\
in vec2 position;                                                                       \n\
flat in vec2 vertices[3];                                                               \n\
flat in vec2 normEdges[3];                                                              \n\
                                                                                        \n\
int minIndex(in vec3 dist)                                                              \n\
{                                                                                       \n\
    int minIdx = (dist[1] < dist[0])? 1: 0;                                             \n\
    if(dist[2] < dist[minIdx])                                                          \n\
        minIdx = 2;                                                                     \n\
                                                                                        \n\
    return minIdx;                                                                      \n\
}                                                                                       \n\
                                                                                        \n\
vec3 computeVertexSqrDist()                                                             \n\
{                                                                                       \n\
    return vec3(                                                                        \n\
        dot(position - vertices[0], position - vertices[0]),                            \n\
        dot(position - vertices[1], position - vertices[1]),                            \n\
        dot(position - vertices[2], position - vertices[2]));                           \n\
}                                                                                       \n\
                                                                                        \n\
vec3 computeEdgeDist()                                                                  \n\
{                                                                                       \n\
    return vec3(                                                                        \n\
        determinant(mat2(normEdges[0], position - vertices[1])),                        \n\
        determinant(mat2(normEdges[1], position - vertices[2])),                        \n\
        determinant(mat2(normEdges[2], position - vertices[0])));                       \n\
}                                                                                       \n\
                                                                                        \n\
float irlerp(in vec2 vx, in vec2 v1, in vec2 v2)                                        \n\
{                                                                                       \n\
    float alpha = acos(clamp(dot(v1, vx), -1., 1.));                                    \n\
    float beta = acos(clamp(dot(v1, v2), -1., 1.));                                     \n\
    return alpha / beta;                                                                \n\
}                                                                                       \n\
                                                                                        \n\
vec4 quadraticInterp(in vec4 colors[6])                                                 \n\
{                                                                                       \n\
    return                                                                              \n\
        colors[0] * linearBasis.x * (2. * linearBasis.x - 1.) +                         \n\
        colors[1] * linearBasis.y * (2. * linearBasis.y - 1.) +                         \n\
        colors[2] * linearBasis.z * (2. * linearBasis.z - 1.) +                         \n\
        colors[3] * 4. * linearBasis.y * linearBasis.z +                                \n\
        colors[4] * 4. * linearBasis.z * linearBasis.x +                                \n\
        colors[5] * 4. * linearBasis.x * linearBasis.y;                                 \n\
}                                                                                       \n\
                                                                                        \n\
float interpFactor(float dist, float radius)                                            \n\
{                                                                                       \n\
    return clamp(.5 - dist*zoom + radius, 0, 1);                                        \n\
}                                                                                       \n\
                                                                                        \n\
vec4 colorWithBordersAndPoints(in vec4 colorNodes[6])                                   \n\
{                                                                                       \n\
    vec3 vertexSqrDist = computeVertexSqrDist();                                        \n\
    int closestVx = minIndex(vertexSqrDist);                                            \n\
                                                                                        \n\
    vec3 edgeDist = computeEdgeDist();                                                  \n\
    int closestEdge = minIndex(edgeDist);                                               \n\
                                                                                        \n\
    vec4 color = quadraticInterp(colorNodes);                                           \n\
                                                                                        \n\
    if(showWireframe)                                                                   \n\
    {                                                                                   \n\
        color = mix(color, vec4(0., 0., 0., 1.),                                        \n\
            interpFactor(edgeDist[closestEdge], halfLineWidth+.5));                     \n\
        color = mix(color, colorNodes[closestEdge + 3]*.5,                              \n\
            interpFactor(edgeDist[closestEdge], halfLineWidth));                        \n\
        color = mix(color, vec4(0., 0., 0., 1.),                                        \n\
            interpFactor(sqrt(vertexSqrDist[closestVx]), pointRadius+.5));              \n\
        color = mix(color, colorNodes[closestVx],                                       \n\
            interpFactor(sqrt(vertexSqrDist[closestVx]), pointRadius));                 \n\
    }                                                                                   \n\
                                                                                        \n\
    return color;                                                                       \n\
}";


static const char* frag_triangle = "                                                    \n\
#version 410 core                                                                       \n\
                                                                                        \n\
uniform samplerBuffer nodes;                                                            \n\
uniform float zoom;                                                                     \n\
uniform float pointRadius;                                                              \n\
uniform float halfLineWidth;                                                            \n\
                                                                                        \n\
in vec3 linearBasis;                                                                    \n\
in vec2 position;                                                                       \n\
flat in vec2 vertices[3];                                                               \n\
flat in vec2 normEdges[3];                                                              \n\
                                                                                        \n\
out vec4 out_color;                                                                     \n\
                                                                                        \n\
int minIndex(in vec3 dist);                                                             \n\
vec3 computeVertexSqrDist();                                                            \n\
vec3 computeEdgeDist();                                                                 \n\
float irlerp(in vec2 vx, in vec2 v1, in vec2 v2);                                       \n\
vec4 quadraticInterp(in vec4 colors[6]);                                                \n\
float interpFactor(float dist, float radius);                                           \n\
vec4 colorWithBordersAndPoints(in vec4 colorNodes[6]);                                  \n\
                                                                                        \n\
int baseVxIndex = gl_PrimitiveID * 6;                                                   \n\
int baseEdgeIndex = baseVxIndex + 3;                                                    \n\
                                                                                        \n\
void main(void)                                                                         \n\
{                                                                                       \n\
    vec4 colorNodes[] = vec4[6](                                                        \n\
        texelFetch(nodes, baseVxIndex + 0),                                             \n\
        texelFetch(nodes, baseVxIndex + 1),                                             \n\
        texelFetch(nodes, baseVxIndex + 2),                                             \n\
        texelFetch(nodes, baseEdgeIndex + 0),                                           \n\
        texelFetch(nodes, baseEdgeIndex + 1),                                           \n\
        texelFetch(nodes, baseEdgeIndex + 2)                                            \n\
        );                                                                              \n\
                                                                                        \n\
    out_color = colorWithBordersAndPoints(colorNodes);                                  \n\
}";

static const char* frag_singular = "                                                    \n\
#version 410 core                                                                       \n\
                                                                                        \n\
uniform samplerBuffer nodes;                                                            \n\
uniform float zoom;                                                                     \n\
uniform float pointRadius;                                                              \n\
uniform float halfLineWidth;                                                            \n\
                                                                                        \n\
in vec3 linearBasis;                                                                    \n\
in vec2 position;                                                                       \n\
flat in vec2 vertices[3];                                                               \n\
flat in vec2 normEdges[3];                                                              \n\
                                                                                        \n\
out vec4 out_color;                                                                     \n\
                                                                                        \n\
int minIndex(in vec3 dist);                                                             \n\
vec3 computeVertexSqrDist();                                                            \n\
vec3 computeEdgeDist();                                                                 \n\
float irlerp(in vec2 vx, in vec2 v1, in vec2 v2);                                       \n\
vec4 quadraticInterp(in vec4 colors[6]);                                                \n\
float interpFactor(float dist, float radius);                                           \n\
vec4 colorWithBordersAndPoints(in vec4 colorNodes[6]);                                  \n\
                                                                                        \n\
int baseVxIndex = gl_PrimitiveID * 7;                                                   \n\
int baseEdgeIndex = baseVxIndex + 4;                                                    \n\
                                                                                        \n\
void main(void)                                                                         \n\
{                                                                                       \n\
    vec4 colorNodes[] = vec4[6](                                                        \n\
        mix(texelFetch(nodes, baseVxIndex + 0),                                         \n\
        texelFetch(nodes, baseVxIndex + 3),                                             \n\
        irlerp(normalize(position - vertices[0]),                                       \n\
        normEdges[2], -normEdges[1])),                                                  \n\
        texelFetch(nodes, baseVxIndex + 1),                                             \n\
        texelFetch(nodes, baseVxIndex + 2),                                             \n\
        texelFetch(nodes, baseEdgeIndex + 0),                                           \n\
        texelFetch(nodes, baseEdgeIndex + 1),                                           \n\
        texelFetch(nodes, baseEdgeIndex + 2)                                            \n\
        );                                                                              \n\
                                                                                        \n\
    out_color = colorWithBordersAndPoints(colorNodes);                                  \n\
}";                                                                                     


class QMeshRenderer
{
public:
    QMeshRenderer() :
        m_verticesBuffer(0), m_triangleIndicesBuffer(0),
        m_singularIndicesBuffer(0), m_triangleNodesBuffer(0),
        m_singularNodesBuffer(0), m_triangleNodesTexture(0),
        m_singularNodesTexture(0), m_pTriangleProgram(0),
        m_pSingularProgram(0),  m_vao(0), m_pQMesh(0)
    {};

    ~QMeshRenderer()
    {
        SAFE_DELETE(m_pSingularProgram);
        SAFE_DELETE(m_pTriangleProgram);
    };

    void render(Eigen::Matrix4f& _viewMatrix, float _zoom = 1.f, float _pointRadius = 2.f, float _lineWidth = 1.f, bool _showShaderWireframe = false);
    bool init(QMesh* _qMesh);
    
    inline void setQMesh(QMesh* _qMesh)
    {
        m_pQMesh = _qMesh;
    }

private:
    bool loadShaders();
    bool initGl();
    void renderTriangles(GLuint _shader, bool _singular = false);

private:
    GLuint m_verticesBuffer;
    GLuint m_triangleIndicesBuffer;
    GLuint m_singularIndicesBuffer;
    GLuint m_triangleNodesBuffer;
    GLuint m_singularNodesBuffer;
    GLuint m_triangleNodesTexture;
    GLuint m_singularNodesTexture;

    GLuint m_vao;

    QMesh* m_pQMesh;

    Technique* m_pTriangleProgram;
    Technique* m_pSingularProgram;
};

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
    GLCheckError();

    m_pTriangleProgram = new Technique();
    if(!m_pTriangleProgram->Init())
    {
        return false;
    }

    bool bRes = true;

    bRes &= m_pTriangleProgram->AddShader(GL_VERTEX_SHADER, vert_common);
    bRes &= m_pTriangleProgram->AddShader(GL_GEOMETRY_SHADER, geom_common);
    bRes &= m_pTriangleProgram->AddShader(GL_FRAGMENT_SHADER, frag_common);
    bRes &= m_pTriangleProgram->AddShader(GL_FRAGMENT_SHADER, frag_triangle);
    assert(bRes);

    bRes &= m_pTriangleProgram->Finalize();
    assert(bRes);


    m_pSingularProgram = new Technique();
    if(!m_pSingularProgram->Init())
    {
        return false;
    }

    bRes &= m_pSingularProgram->AddShader(GL_VERTEX_SHADER, vert_common);
    bRes &= m_pSingularProgram->AddShader(GL_GEOMETRY_SHADER, geom_common);
    bRes &= m_pSingularProgram->AddShader(GL_FRAGMENT_SHADER, frag_common);
    bRes &= m_pSingularProgram->AddShader(GL_FRAGMENT_SHADER, frag_singular);
    assert(bRes);

    bRes &= m_pSingularProgram->Finalize();
    assert(bRes);

    return GLCheckError();;
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
        Technique* program = (pass == 0) ? m_pTriangleProgram : m_pSingularProgram;

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
#endif