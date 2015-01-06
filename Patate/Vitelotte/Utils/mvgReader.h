
#ifndef _MVGREADER_H_
#define _MVGREADER_H_


#include <cassert>
#include <stdexcept>
#include <istream>
#include <string>
#include <sstream>

#include "../../common/surface_mesh/objReader.h"


namespace Vitelotte
{


//class QVGReadError : public std::runtime_error
//{
//public:
//    inline QVGReadError(const std::string& _what)
//        : std::runtime_error(_what)
//    {}
//};


/**
 * \brief The QVGReader class allow to read QMesh objects from `.qvg` files.
 *
 * There is an example of qvg file:
 * \code{.qvg}
 * qvg 1.0
 * 4 10 0 2
 * v 0 0
 * v 1 0
 * v 1 1
 * v 0 1
 * n 1   0   0   1
 * n 0.8 0.2 0   1
 * n 0   1   0   1
 * n 0   0.5 0.5 1
 * n 0   0   1   1
 * n 0.5 0   0.5 1
 * n 0.5 1   0   1
 * n 1   1   0   1
 * n 0.5 1   0.5 1
 * n 0   1   1   1
 * f 0/0 1/2 2/4 3 5 1
 * fs 2/4 3/6 0/9/0 8 5 6
 * \endcode
 * The syntax of qwg files is highly inspired by the obj file format.
 *
 * The first line `qvg 1.0` is required. For the time being, v1.0 is the only
 * version of the qvg file format, but this may of course change.
 *
 * The second line gives, in order, the number of vertices, nodes, curves and
 * faces (triangles). These number *must* match the content of the file, or
 * a QVGReadError will be thrown. The rest of the file is composed of 4 blocs
 * corresponding to these four declarations. Blocs should be declared in order
 * and can not be mixed.
 *
 * \todo Support 3D coordinates
 *
 * The vertex bloc is composed of simple declaration of the form `v x y` where
 * `x` and `y` are the coordinate of the vertex.
 *
 * \todo Specify / create a syntax to specify coloc space ?
 *
 * The node bloc is similar to the vertex bloc `n r g b a`, where `r`, `g`, `b`
 * and `a` are the red, green, blue and alpha color component. A node is simply
 * a color, so the same one can be used for two completly unrelated vectices if
 * they happen to have the same color. However, doing so may complicate some
 * use case, like color editing.
 *
 * \todo Document curves
 *
 * The face bloc is the most complicated. There are two kind of faces: normals
 * and singulars. Normal faces link 3 vertices to form a triangle and 6 nodes
 * that specify colors on vertices and edge midpoints. The syntax is
 * \code{.qvg}
 * f v0/nv0 v1/nv1 v2/nv2 ne0 ne1 ne2
 * \endcode
 * where `vx` is the index of the vertex `x`, `nvx` is the index of the node
 * attached to the vertex `x` and `nex` is the node attache to the edge opposed
 * to the vertex `x`.
 *
 * Singular triangles have a 7th node attached to a vertex. The syntax is the
 * same as normal faces, except for the prefix `fs` and the fact that one of
 * the vertex must have a 2nd node:
 * \code
 * f v0/nv0 v1/nv1/nvs v2/nv2 ne0 ne1 ne2
 * \endcode
 * where `nvs` is the second node of the vertex.
 *
 * \see QMesh QVGWriter
 */
template < typename _Mesh >
class MVGReader: public PatateCommon::OBJReader<typename _Mesh::Vector>
{
public:
    typedef _Mesh Mesh;
    typedef PatateCommon::OBJReader<typename _Mesh::Vector> Base;

    typedef typename Mesh::Vector Vector;
    typedef typename Mesh::NodeValue NodeValue;

public:

    /**
     * \brief Default constructor
     */
    inline MVGReader(Mesh& mesh);

    using Base::error;
    using Base::warning;
    using Base::parseIndiceList;

protected:
    virtual void parseHeader(std::istream& in);
    virtual bool parseDefinition(const std::string& spec,
                                 std::istream& def);

protected:

    using Base::m_mesh;
    using Base::m_fVertices;

    std::string m_tmp;
    std::vector<unsigned> m_faceIndices;
};


template < typename Mesh >
void readMvg(std::istream& in, Mesh& mesh);

template < typename Mesh >
void readMvgFromFile(const std::string& filename, Mesh& mesh);


} // namespace Vitelotte

#include "mvgReader.hpp"


#endif // _MVGREADER_H_

