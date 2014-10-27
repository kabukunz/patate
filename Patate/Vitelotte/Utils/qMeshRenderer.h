#ifndef _QMESH_RENDERER_BASE_H_
#define _QMESH_RENDERER_BASE_H_

#include <GL/glew.h>

#include "../Core/qMesh.h"
#include "../../common/gl_utils/shader.h"


namespace Vitelotte {

#include "shaders.hpp"

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

    Patate::Shader* m_pTriangleProgram;
    Patate::Shader* m_pSingularProgram;
};


#include "qMeshRenderer.hpp"

}

#endif
