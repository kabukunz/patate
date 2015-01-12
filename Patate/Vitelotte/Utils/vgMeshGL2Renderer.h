#ifndef _VG_MESH_GL2_RENDERER_BASE_H_
#define _VG_MESH_GL2_RENDERER_BASE_H_


#include <Eigen/Dense>

#include "../../common/gl_utils/shader.h"

#include "../Core/femUtils.h"
#include "shaders.hpp"


namespace Vitelotte {


class VGMeshGL2RendererShader
{
public:
    enum TriangleType { Standard, Singular };

public:
    inline VGMeshGL2RendererShader()
        : m_verticesLoc(-1), m_basisLoc(-1),
          m_edgesLoc(-1), m_baseNodeCoordLoc(-1)
    {}

    virtual bool useShader(TriangleType triangleType) = 0;
    virtual void setNodesTexture(int texUnit, int size) = 0;

    inline GLint verticesAttibLocation() { return m_verticesLoc; }
    inline GLint basisAttibLocation() { return m_basisLoc; }
    inline GLint edgesAttibLocation() { return m_edgesLoc; }
    inline GLint baseNodeCoordAttibLocation() { return m_baseNodeCoordLoc; }

private:
    VGMeshGL2RendererShader(const VGMeshGL2RendererShader&);

protected:
    GLint m_verticesLoc;
    GLint m_basisLoc;
    GLint m_edgesLoc;
    GLint m_baseNodeCoordLoc;
};


class VGMeshRendererGL2DefaultShader : public VGMeshGL2RendererShader
{
public:
    inline VGMeshRendererGL2DefaultShader();
    virtual inline ~VGMeshRendererGL2DefaultShader() {}

    virtual bool useShader(TriangleType triangleType);
    virtual void setNodesTexture(int texUnit, int size);

    inline Eigen::Matrix4f& viewMatrix() { return m_viewMatrix; }
    inline const Eigen::Matrix4f& viewMatrix() const { return m_viewMatrix; }
    inline void setViewMatrix(const Eigen::Matrix4f& viewMatrix)
        { m_viewMatrix = viewMatrix; }


protected:
    inline void getUniforms();
    inline void setupUniforms();

protected:
    PatateCommon::Shader m_shader;

    GLint m_viewMatrixLoc;
    GLint m_nodesLoc;
    GLint m_singularTrianglesLoc;
    GLint m_nodesTextureSizeLoc;

    Eigen::Matrix4f m_viewMatrix;
    int m_nodes;
    bool m_singularTriangles;
    int m_textureSize;
};


class VGMeshRendererGL2WireframeShader : public VGMeshGL2RendererShader
{
public:
    inline VGMeshRendererGL2WireframeShader();
    virtual inline ~VGMeshRendererGL2WireframeShader() {}

    virtual bool useShader(TriangleType triangleType);
    virtual void setNodesTexture(int texUnit, int size);

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
class VGMeshGL2Renderer
{
public:
    typedef _Mesh Mesh;

    typedef typename Mesh::Node Node;
    typedef typename Mesh::Vector Vector;
    typedef typename Mesh::NodeValue NodeValue;

public:
    VGMeshGL2Renderer() :
        m_verticesBuffer(0),
        /*m_nodesBuffer(0),*/ m_nodesTexture(0),
        /*m_vao(0),*/ m_pMesh(0)
    {}

    ~VGMeshGL2Renderer() {}

    void initialize(Mesh* _mesh=0);
    void render(VGMeshGL2RendererShader& shaders);

    inline void setMesh(Mesh* _mesh)
    {
        m_pMesh = _mesh;
        updateMesh();
    }

    void updateMesh();

private:
    typedef std::vector<NodeValue> NodesVector;

    struct Vertex {
        Vector pos;
        Eigen::Vector2f edge0;
        Eigen::Vector2f edge1;
        Eigen::Vector2f baseNodeCoord;
        unsigned basis;
    };
    typedef std::vector<Vertex> VerticesVector;

private:
    void renderTriangles(VGMeshGL2RendererShader& shaders, bool _singular = false);

    NodeValue nodeValue(Node node) const;

    template < typename T >
    void createAndUploadBuffer(GLuint& glId, GLenum type,
                               const std::vector<T>& data,
                               GLenum usage = GL_DYNAMIC_DRAW);

private:
    GLuint m_verticesBuffer;
//    GLuint m_nodesBuffer;
    GLuint m_nodesTexture;

//    GLuint m_vao;

    Mesh* m_pMesh;

    VerticesVector m_vertices;

    NodesVector m_nodes;
    int m_nTriangles;
    int m_nSingulars;
    int m_nodeTextureSize;
};


}

#include "vgMeshGL2Renderer.hpp"


#endif
