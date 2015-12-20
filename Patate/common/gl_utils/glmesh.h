/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _PATATE_COMMON_GL_UTILS_GLMESH_H
#define _PATATE_COMMON_GL_UTILS_GLMESH_H

#ifdef __APPLE__
    #include <OpenGL/gl3.h>
    #include <OpenGL/gl3ext.h>

    #define __gl_h_

#elif defined _MSC_VER
    #include <Windows.h>
    #include <GL/gl.h>

#else
    #define GL_GLEXT_PROTOTYPES

    #include <GL/gl.h>
    #include <GL/glext.h>
#endif

#include "Eigen/Dense"
#include "Patate/common/surface_mesh/surfaceMesh.h"
#include <vector>


namespace PatateCommon
{
/*!
 * \brief The GL3DMesh struct implements basic VBO display for triangular 3d meshes.
 *
 * It can be generated by OBJReader
 */
struct GLTri3DMesh{
    enum { Dim = 3 };
    typedef GLfloat                       Scalar;
    typedef Eigen::Matrix<Scalar, Dim, 1> Vector;
    typedef SurfaceMesh::Vertex           Vertex; // ids

    //! \brief Embedded Point type compatible with Grenaille API
    struct GrenaillePoint
    {
    public:
        enum {Dim = GLTri3DMesh::Dim};
        typedef typename GLTri3DMesh::Scalar Scalar;
        typedef Eigen::Matrix<Scalar, Dim,   1>   VectorType;
        typedef Eigen::Matrix<Scalar, Dim+1, 1>   HVectorType;
        typedef Eigen::Matrix<Scalar, Dim, Dim>   MatrixType;

        inline GrenaillePoint(int id, GLTri3DMesh &mesh)
            : m_pos   (mesh.getVertexMap(id)),
              m_normal(mesh.getNormalVectorMap(id)) {}

        inline const Eigen::Map< VectorType >& pos()    const { return m_pos; }
        inline const Eigen::Map< VectorType >& normal() const { return m_normal; }
        inline Eigen::Map< VectorType >& pos()    { return m_pos; }
        inline Eigen::Map< VectorType >& normal() { return m_normal; }
    private:
        Eigen::Map< VectorType > m_pos, m_normal;
    };


    inline void addFace(const std::vector<Vertex>& vertices);
    inline void addVertex(const Vector& v);
    inline unsigned int nVertices() const;

    inline GLTri3DMesh();
    inline void initVBO(bool initForPicking = true);
    inline void draw();
    // draw ids for picking
    inline void drawIds();

    inline Eigen::Map<Vector> getVertexMap(int id);
    inline Eigen::Map<Vector> getNormalVectorMap(int id);

protected:
    inline void computeNormals();

    typedef std::vector<Scalar> VContainer;
    typedef std::vector<GLuint> FContainer;

    VContainer _vertices;
    VContainer _normals;
    FContainer _faces;    //will be the index buffer
    bool _init, _pickingInit;

    GLuint _vao;
    GLuint _vboFaceArray, _vboVertArray, _vboNormalArray, _vboIdsArray;
};


} // namespace PatateCommon

#include "glmesh.hpp"

#endif // _PATATE_COMMON_GL_UTILS_GLMESH_H

