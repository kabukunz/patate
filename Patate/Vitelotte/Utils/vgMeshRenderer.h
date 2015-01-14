#ifndef _VITELOTTE_VG_MESH_RENDERER_
#define _VITELOTTE_VG_MESH_RENDERER_


#include <Eigen/Dense>

#include "../../common/gl_utils/shader.h"
#include "../../common/gl_utils/color.h"

#include "../Core/femUtils.h"
#include "shaders.hpp"


namespace Vitelotte {


class VGMeshRendererShader
{
public:
    enum TriangleType {
        Quadratic = 0x01,
        Singular  = 0x02
    };

public:
    inline VGMeshRendererShader() {}

    virtual bool useShader(TriangleType triangleType) = 0;
    virtual void setNodesTexture(TriangleType triangleType, int texUnit, int baseOffset) = 0;

    virtual GLint verticesAttibLocation(TriangleType triangleType) = 0;

private:
    VGMeshRendererShader(const VGMeshRendererShader&);
};


class VGMeshRendererDefaultShader : public VGMeshRendererShader
{
public:
    inline VGMeshRendererDefaultShader();
    virtual inline ~VGMeshRendererDefaultShader() {}

    virtual GLint verticesAttibLocation(TriangleType triangleType);

    virtual bool useShader(TriangleType triangleType);
    virtual void setNodesTexture(TriangleType triangleType, int texUnit, int baseOffset);

    inline Eigen::Matrix4f& viewMatrix() { return m_viewMatrix; }
    inline const Eigen::Matrix4f& viewMatrix() const { return m_viewMatrix; }
    inline void setViewMatrix(const Eigen::Matrix4f& viewMatrix)
        { m_viewMatrix = viewMatrix; }

protected:
    struct Uniforms
    {
        GLint viewMatrixLoc;
        GLint nodesLoc;
        GLint baseNodeIndexLoc;
        GLint singularTrianglesLoc;
    };

protected:
    inline void getUniforms(PatateCommon::Shader& shader, Uniforms& uniforms);
    inline void setupUniforms(const Uniforms& uniforms);

protected:
    PatateCommon::Shader m_linearShader;
    PatateCommon::Shader m_quadraticShader;

    GLint m_verticesLocLinear;
    GLint m_verticesLocQuadratic;

    Uniforms m_linearUniforms;
    Uniforms m_quadraticUniforms;

    Eigen::Matrix4f m_viewMatrix;
    int m_nodes;
    int m_baseNodeIndex;
    bool m_singularTriangles;
};


class VGMeshRendererWireframeShader : public VGMeshRendererShader
{
public:
    inline VGMeshRendererWireframeShader();
    virtual inline ~VGMeshRendererWireframeShader() {}

    virtual GLint verticesAttibLocation(TriangleType triangleType);

    virtual bool useShader(TriangleType triangleType);
    virtual void setNodesTexture(TriangleType triangleType, int texUnit, int baseOffset);

    inline Eigen::Matrix4f& viewMatrix() { return m_viewMatrix; }
    inline const Eigen::Matrix4f& viewMatrix() const { return m_viewMatrix; }
    inline void setViewMatrix(const Eigen::Matrix4f& viewMatrix)
        { m_viewMatrix = viewMatrix; }

    inline float zoom() const { return m_zoom; }
    inline void setZoom(float zoom) { m_zoom = zoom; }

    inline float lineWidth() const { return m_lineWidth; }
    inline void setLineWidth(float lineWidth) { m_lineWidth = lineWidth; }

    inline const Eigen::Vector4f& wireframeColor() const { return m_wireframeColor; }
    inline void setWireframeColor(const Eigen::Vector4f& color)
        { m_wireframeColor = color; }


protected:
    inline void getUniforms();
    inline void setupUniforms();

protected:
    PatateCommon::Shader m_shader;

    GLint m_verticesLoc;

    GLint m_viewMatrixLoc;
    GLint m_zoomLoc;
    GLint m_lineWidthLoc;
    GLint m_wireframeColorLoc;

    Eigen::Matrix4f m_viewMatrix;
    float m_zoom;
    float m_lineWidth;
    Eigen::Vector4f m_wireframeColor;
};


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
        m_verticesBuffer(0), m_indicesBuffer(0),
        m_nodesBuffer(0), m_nodesTexture(0),
        m_vao(0), m_pMesh(0)
    {}

    ~VGMeshRenderer() {}

    void initialize(Mesh* _mesh=0);
    void render(VGMeshRendererShader& shaders);

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
    void renderTriangles(VGMeshRendererShader& shaders, bool _singular = false);

    NodeValue nodeValue(Node node) const;

    template < typename T >
    void createAndUploadBuffer(GLuint& glId, GLenum type,
                               const std::vector<T>& data,
                               GLenum usage = GL_DYNAMIC_DRAW);

private:
    GLuint m_verticesBuffer;
    GLuint m_indicesBuffer;
    GLuint m_nodesBuffer;
    GLuint m_nodesTexture;

    GLuint m_vao;

    Mesh* m_pMesh;

    VectorsVector m_vertices;

    IndicesVector m_indices;

    bool m_quadratic;
    NodesVector m_nodes;
    int m_nTriangles;
    int m_nSingulars;
};


}

#include "vgMeshRenderer.hpp"


#endif
