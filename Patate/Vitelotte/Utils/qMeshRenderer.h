#ifndef _QMESH_RENDERER_BASE_H_
#define _QMESH_RENDERER_BASE_H_

#include <GL/glew.h>

#include "../Core/qMesh.h"
#include "../../common/gl_utils/shader.h"


namespace Vitelotte {

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
        PATATE_SAFE_DELETE(m_pSingularProgram);
        PATATE_SAFE_DELETE(m_pTriangleProgram);
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

    Shader* m_pTriangleProgram;
    Shader* m_pSingularProgram;
};


#include "qMeshRenderer.hpp"

}

#endif
