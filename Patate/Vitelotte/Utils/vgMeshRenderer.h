#ifndef _VG_MESH_RENDERER_BASE_H_
#define _VG_MESH_RENDERER_BASE_H_


#include <Eigen/Dense>

#include "../../common/gl_utils/shader.h"

#include "shaders.hpp"


namespace Vitelotte {


inline Eigen::Vector4f linearToSrgb(const Eigen::Vector4f& linear);
inline Eigen::Vector4f srgbToLinear(const Eigen::Vector4f& srgb);


class VGMeshRendererShader
{
public:
    enum TriangleType { Standard, Singular };

public:
    inline VGMeshRendererShader() {}

    virtual bool useShader(TriangleType triangleType) = 0;
    virtual void setNodesTexture(int texUnit, int baseOffset) = 0;

    inline GLint verticesAttibLocation() { return m_verticesLoc; }

private:
    VGMeshRendererShader(const VGMeshRendererShader&);

protected:
    GLint m_verticesLoc;
};


class VGMeshRendererDefaultShader : public VGMeshRendererShader
{
public:
    inline VGMeshRendererDefaultShader();
    virtual inline ~VGMeshRendererDefaultShader() {}

    virtual bool useShader(TriangleType triangleType);
    virtual void setNodesTexture(int texUnit, int baseOffset);

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
    GLint m_baseNodeIndexLoc;
    GLint m_singularTrianglesLoc;

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

    virtual bool useShader(TriangleType triangleType);
    virtual void setNodesTexture(int texUnit, int baseOffset);

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

//class VGMeshRendererDefaultShaders : public VGMeshRendererShaders
//{
//public:
//    inline VGMeshRendererDefaultShaders();
//    virtual inline ~VGMeshRendererDefaultShaders() {}

//    virtual GLuint useShader(TriangleType triangleType);
//    virtual void setColorTexture(int texUnit, int baseOffset);

//    inline Eigen::Matrix4f& viewMatrix() { return m_viewMatrix; }
//    inline const Eigen::Matrix4f& viewMatrix() const { return m_viewMatrix; }

//    inline bool& showWireframe() { return m_showWireframe; }
//    inline const bool& showWireframe() const { return m_showWireframe; }

//    inline float& zoom() { return m_zoom; }
//    inline const float& zoom() const { return m_zoom; }

//    inline float& pointRadius() { return m_pointRadius; }
//    inline const float& pointRadius() const { return m_pointRadius; }

//    inline float& lineWidth() { return m_lineWidth; }
//    inline const float& lineWidth() const { return m_lineWidth; }

//    inline Eigen::Vector4f& wireframeColor() { return m_wireframeColor; }
//    inline const Eigen::Vector4f& wireframeColor() const { return m_wireframeColor; }

//    inline Eigen::Vector4f& pointColor() { return m_pointColor; }
//    inline const Eigen::Vector4f& pointColor() const { return m_pointColor; }


//protected:
//    struct Uniforms
//    {
//        GLint viewMatrixLoc;
//        GLint showWireframeLoc;
//        GLint zoomLoc;
//        GLint pointRadiusLoc;
//        GLint halfLineWidthLoc;
//        GLint wireframeColorLoc;
//        GLint pointColorLoc;
//    };

//    inline void getUniforms(PatateCommon::Shader& shader, Uniforms& uniforms);
//    inline void setupUniforms(const Uniforms& uniforms);

//protected:
//    PatateCommon::Shader m_shader;

//    Uniforms m_uniforms;

//    Eigen::Matrix4f m_viewMatrix;
//    bool m_showWireframe;
//    float m_zoom;
//    float m_pointRadius;
//    float m_lineWidth;
//    Eigen::Vector4f m_wireframeColor;
//    Eigen::Vector4f m_pointColor;
//};


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

    NodesVector m_nodes;
    int m_nTriangles;
    int m_nSingulars;
};


}

#include "vgMeshRenderer.hpp"


#endif
