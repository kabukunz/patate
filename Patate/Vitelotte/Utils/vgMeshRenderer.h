/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _VITELOTTE_VG_MESH_RENDERER_
#define _VITELOTTE_VG_MESH_RENDERER_


#include <Eigen/Dense>

#include "../../common/gl_utils/shader.h"
#include "../../common/gl_utils/color.h"

#include "../Core/femUtils.h"
#include "vgMeshRendererShaders.hpp"


namespace Vitelotte {


template < typename Vector >
struct DefaultPosProj {
    inline Eigen::Vector4f operator()(const Vector& position) const;
};

template < typename Value >
struct DefaultValueProj {
    inline Eigen::Vector4f operator()(const Value& value) const;
};

template < class _Mesh,
           typename _PosProj   = DefaultPosProj  <typename _Mesh::Vector>,
           typename _ValueProj = DefaultValueProj<typename _Mesh::Value> >
class VGMeshRenderer
{
public:
    typedef _Mesh      Mesh;
    typedef _PosProj   PosProj;
    typedef _ValueProj ValueProj;

    typedef typename Mesh::Node Node;
    typedef typename Mesh::Vector Vector;
    typedef typename Mesh::Value Value;

    typedef typename Mesh::Vertex Vertex;
    typedef typename Mesh::Face Face;

    typedef Eigen::Vector4f Vector4;

    enum
    {
        NORMAL_TRIANGLES = 0x01,
        SINGULAR_TRIANGLES = 0x02,

        ALL_TRIANGLES = NORMAL_TRIANGLES | SINGULAR_TRIANGLES
    };

    enum
    {
        VG_MESH_POSITION_ATTR_LOC
    };

    enum
    {
        NODES_TEXTURE_UNIT
    };

public:
    VGMeshRenderer(const PosProj&   posProj   = PosProj(),
                   const ValueProj& valueProj = ValueProj());
    ~VGMeshRenderer();

    bool convertSrgbToLinear() const;
    void setConvertSrgbToLinear(bool enable);

    const PosProj&   positionProjection() const;
          PosProj&   positionProjection();
    const ValueProj& valueProjection()    const;
          ValueProj& valueProjection();

    bool initialize();
    void releaseGLRessources();
    void updateBuffers(const Mesh& mesh);

    void drawGeometry(unsigned geomFlags);
    void render(const Eigen::Matrix4f& viewMatrix);
    void renderWireframe(const Eigen::Matrix4f& viewMatrix,
                         float zoom, float lineWidth = 1,
                         const Eigen::Vector4f& color = Eigen::Vector4f(0, 0, 0, 1));

private:
    typedef std::vector<unsigned> IndicesVector;
    typedef std::vector<Vector4> Vector4Vector;

    struct SolidUniforms
    {
        GLint viewMatrixLoc;
        GLint nodesLoc;
        GLint baseNodeIndexLoc;
        GLint singularTrianglesLoc;
    };

    struct WireframeUniforms
    {
        GLint viewMatrixLoc;
        GLint zoomLoc;
        GLint lineWidthLoc;
        GLint wireframeColorLoc;
    };

protected:
//    void renderTriangles(VGMeshRendererShader& shaders, bool _singular = false);
    bool initSolidShader(PatateCommon::Shader& shader, SolidUniforms& unif,
                         const char *fragCode);
    bool initWireframeShader();

    Vector4 position(const Mesh& mesh, Vertex vx) const;
    Vector4 color(const Mesh& mesh, Node node) const;

    template < typename T >
    void createAndUploadBuffer(GLuint& glId, GLenum type,
                               const std::vector<T>& data,
                               GLenum usage = GL_DYNAMIC_DRAW);

private:
    bool m_initialized;
    bool m_useVao;
    bool m_convertSrgbToLinear;

    PosProj   m_positionProjection;
    ValueProj m_valueProjection;
    Vector4   m_invalidNodeColor;

    PatateCommon::Shader m_solidLinearShader;
    PatateCommon::Shader m_solidQuadraticShader;
    PatateCommon::Shader m_wireframeShader;

    SolidUniforms     m_solidLinearUniforms;
    SolidUniforms     m_solidQuadraticUniforms;
    WireframeUniforms m_wireframeUniforms;

    GLuint m_verticesBuffer;
    GLuint m_indicesBuffer;
    GLuint m_nodesBuffer;
    GLuint m_nodesTexture;

    GLuint m_vao;

    Vector4Vector m_vertices;
    IndicesVector m_indices;
    Vector4Vector m_nodes;

    bool     m_quadratic;
    unsigned m_nTriangles;
    unsigned m_nSingulars;
};


}

#include "vgMeshRenderer.hpp"


#endif
