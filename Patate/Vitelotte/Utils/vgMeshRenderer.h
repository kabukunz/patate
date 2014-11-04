#ifndef _VG_MESH_RENDERER_BASE_H_
#define _VG_MESH_RENDERER_BASE_H_


#include "../../common/gl_utils/shader.h"

#include "../Core/femMesh.h"


namespace Vitelotte {

#include "shaders.hpp"

template < class _Mesh >
class VGMeshRenderer
{
public:
    typedef _Mesh Mesh;

    typedef typename Mesh::Node Node;
    typedef typename Mesh::Vector Vector;
    typedef typename Mesh::NodeValue NodeValue;

public:
    VGMeshRenderer() :
        m_verticesBuffer(0), m_triangleIndicesBuffer(0),
        m_singularIndicesBuffer(0), m_triangleNodesBuffer(0),
        m_singularNodesBuffer(0), m_triangleNodesTexture(0),
        m_singularNodesTexture(0), m_pTriangleProgram(0),
        m_pSingularProgram(0),  m_vao(0), m_pMesh(0)
    {}

    ~VGMeshRenderer()
    {
        PATATE_SAFE_DELETE(m_pSingularProgram);
        PATATE_SAFE_DELETE(m_pTriangleProgram);
    }

    void render(Eigen::Matrix4f& _viewMatrix, float _zoom = 1.f,
                float _pointRadius = 2.f, float _lineWidth = 1.f,
                bool _showShaderWireframe = false);
    bool init(Mesh* _mesh);

    inline void setMesh(Mesh* _mesh)
    {
        m_pMesh = _mesh;
        updateMesh();
    }

    void updateMesh();

private:
    typedef std::vector<unsigned> IndicesVector;
    typedef std::vector<Vector> VectorsVector;
    typedef std::vector<NodeValue> NodesVector;

private:
    bool loadShaders();
    bool initGl();
    void renderTriangles(GLuint _shader, bool _singular = false);

    NodeValue nodeValue(Node node) const;

    template < typename T >
    void createAndUploadBuffer(GLuint& glId, GLenum type,
                               const std::vector<T>& data,
                               GLenum usage = GL_DYNAMIC_DRAW);

private:
    GLuint m_verticesBuffer;
    GLuint m_triangleIndicesBuffer;
    GLuint m_singularIndicesBuffer;
    GLuint m_triangleNodesBuffer;
    GLuint m_singularNodesBuffer;
    GLuint m_triangleNodesTexture;
    GLuint m_singularNodesTexture;

    GLuint m_vao;

    Mesh* m_pMesh;

    Patate::Shader* m_pTriangleProgram;
    Patate::Shader* m_pSingularProgram;

    VectorsVector m_vertices;

    IndicesVector m_triangleIndices;
    IndicesVector m_singularIndices;

    NodesVector m_triangleNodes;
    NodesVector m_singularNodes;
};


}

#include "vgMeshRenderer.hpp"


#endif
