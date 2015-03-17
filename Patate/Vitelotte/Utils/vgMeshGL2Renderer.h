/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _VITELOTTE_VG_MESH_GL2_RENDERER_
#define _VITELOTTE_VG_MESH_GL2_RENDERER_


#include <Eigen/Dense>

#include "../../common/gl_utils/shader.h"

#include "../Core/femUtils.h"
#include "vgMeshGL2RendererShaders.hpp"


namespace Vitelotte {


enum
{
    VG_MESH_GL2_POSITION_ATTR_LOC,
    VG_MESH_GL2_BASIS_ATTR_LOC,
    VG_MESH_GL2_EDGES_ATTR_LOC,
    VG_MESH_GL2_BASE_NODE_COORD_ATTR_LOC,
};

class VGMeshGL2RendererShader
{
public:
    enum TriangleType { Standard, Singular };

public:
    inline VGMeshGL2RendererShader() {}

    virtual bool useShader(TriangleType triangleType) = 0;
    virtual void setNodesTexture(int texUnit, int size) = 0;

private:
    VGMeshGL2RendererShader(const VGMeshGL2RendererShader&);
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
    typedef typename Mesh::Value Value;

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
    typedef std::vector<Value> NodesVector;

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

    Value value(Node node) const;

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
