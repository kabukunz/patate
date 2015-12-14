/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _PATATE_COMMON_GL_UTILS_GLMESH_H
#include "glmesh.h"
#endif



namespace PatateCommon
{

GLTri3DMesh::GLTri3DMesh()
    : _init(false),
      _vboFaceArray(0),
      _vboVertArray(0){

}

void GLTri3DMesh::addFace(const std::vector<Vertex> &vertices){
    assert(vertices.size() == 3);

    _init = false;

    int off = _faces.size();
    _faces.resize(off + 3);

    for(int i = 0; i < 3; ++i) _faces[off+i] = vertices[i].idx();
}

void GLTri3DMesh::addVertex(const Vector &v){
    _init = false;
    std::copy (v.data(),v.data()+Dim,back_inserter(_vertices));
}

unsigned int GLTri3DMesh::nVertices() const{
    return _vertices.size();
}


void GLTri3DMesh::initVBO(){
    if (! _init){
        if(_vertices.empty() || _faces.empty()){
            _init = true;
            return;
        }
        computeNormals();

        glGenVertexArrays(1, &_vao);
        glBindVertexArray(_vao);

        // vertices
        glGenBuffers(1, &_vboVertArray);
        glBindBuffer(GL_ARRAY_BUFFER, _vboVertArray);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Scalar)*_vertices.size(), &(_vertices.front()), GL_STATIC_DRAW);
        glVertexAttribPointer( 0, Dim, GL_FLOAT, GL_FALSE, 0, (GLvoid*)(0) );
        glEnableVertexAttribArray( 0 );

        // vertices
        glGenBuffers(1, &_vboNormalArray);
        glBindBuffer(GL_ARRAY_BUFFER, _vboNormalArray);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Scalar)*_normals.size(), &(_normals.front()), GL_STATIC_DRAW);
        glVertexAttribPointer( 1, Dim, GL_FLOAT, GL_FALSE, 0, (GLvoid*)(0) );
        glEnableVertexAttribArray( 1 );

        // indices
        glGenBuffers(1, &_vboFaceArray);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _vboFaceArray);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint)*_faces.size(), &(_faces.front()), GL_STATIC_DRAW);

        glBindVertexArray(0);

        _init = true;
    }
}

void GLTri3DMesh::draw(){
    if (! _faces.empty()) {
        if( ! _init ) initVBO();

        glBindVertexArray(_vao);
        glDrawElements(GL_TRIANGLES, _faces.size(), GL_UNSIGNED_INT, NULL);
        glBindVertexArray(0);

    }
}

void GLTri3DMesh::computeNormals(){
    _normals.clear();
    _normals.resize(_vertices.size(), Scalar(0));

    typedef Eigen::Map< Vector > VMap;

    // naive computation: just sum up all the faces contributions per vertex
    for(unsigned int i = 0; i < _faces.size(); i += 3){

        VMap v0 (&(_vertices[3*_faces[i  ]]));
        VMap v1 (&(_vertices[3*_faces[i+1]]));
        VMap v2 (&(_vertices[3*_faces[i+2]]));

        Vector n = (v1 - v0).cross((v2 - v0)).normalized();

        VMap (&(_normals[3*_faces[i  ]])) += n;
        VMap (&(_normals[3*_faces[i+1]])) += n;
        VMap (&(_normals[3*_faces[i+2]])) += n;
    }

    for(unsigned int i = 0; i < _normals.size(); i += 3){
        VMap (&(_normals[i])).normalize();
    }
}

} // namespace PatateCommon
