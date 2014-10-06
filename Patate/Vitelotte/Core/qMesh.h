#ifndef _QMESH_H_
#define _QMESH_H_

#include <cassert>
#include <stdexcept>
#include <vector>
#include <fstream>

#include <Eigen/Dense>


namespace Vitelotte
{

struct QuadraticTriangle
{
    inline const unsigned& vertex(unsigned _i) const
    {
        assert(_i < 3);
        return m_vertices[_i];
    }

    inline unsigned& vertex(unsigned _i)
    {
        assert(_i < 3);
        return m_vertices[_i];
    }

    inline const unsigned& vxNode(unsigned _i) const
    {
        assert(_i < 3);
        return m_nodes[_i];
    }

    inline unsigned& vxNode(unsigned _i)
    {
        assert(_i < 3);
        return m_nodes[_i];
    }

    // Edge i is the one opposed to vertex i
    inline const unsigned& edgeNode(unsigned _i) const
    {
        assert(_i < 3);
        return m_nodes[_i+3];
    }

    inline unsigned& edgeNode(unsigned _i)
    {
        assert(_i < 3);
        return m_nodes[_i+3];
    }

    unsigned m_vertices[3];
    unsigned m_nodes[6];
};

struct SingularQuadraticTriangle
{
    inline const unsigned& vertex(unsigned _i) const
    {
        assert(_i < 3);
        return m_vertices[_i];
    }

    inline unsigned& vertex(unsigned _i)
    {
        assert(_i < 3);
        return m_vertices[_i];
    }

    // The singular vertex is always vx 0. Node 1 & 2 correspond to vx 1 & 2.
    inline const unsigned& vxNode(unsigned _i) const
    {
        assert(_i < 4);
        return m_nodes[_i];
    }

    inline unsigned& vxNode(unsigned _i)
    {
        assert(_i < 4);
        return m_nodes[_i];
    }

    inline const unsigned& edgeNode(unsigned _i) const
    {
        assert(_i < 3);
        return m_nodes[_i+4];
    }

    inline unsigned& edgeNode(unsigned _i)
    {
        assert(_i < 3);
        return m_nodes[_i+4];
    }

    unsigned m_vertices[3];
    unsigned m_nodes[7];
};

/**
 * \brief The QMesh class represent a triangular mesh with quadratic color
 * interpolation over each triangle.
 *
 * A color value is set on vertices and edge midpoints per triangle. This mean
 * that neighbor triangles can have different colors defined for their common
 * edge, thus creating a color discontinuity.
 *
 * QMesh also allow to represent singular triangles. Singular triangles have
 * two colors defined on the vertex 0, one for each edge, and do a radial
 * interpolation between them.
 *
 * \todo Add some illustrations.
 *
 * \see QMeshRenderer
 */
class QMesh
{
public:
    typedef std::pair<Eigen::Vector2f, Eigen::Vector2f> Curve;

    // TODO: Move to 3D vectors.
    typedef std::vector<Eigen::Vector2f> VertexList;
    typedef std::vector<Eigen::Vector4f> NodeList;
    typedef std::vector<Curve> CurveList;
    typedef std::vector<QuadraticTriangle> TriangleList;
    typedef std::vector<SingularQuadraticTriangle> SingularTriangleList;

public:
    QMesh() : 
        m_valid(true), m_vertices(), m_nodes(), m_curves(),
        m_triangles(), m_singularTriangles(), m_boundingBox()
    {}

    ~QMesh() {}

    void dumpQvg(std::ostream& _out);

public:
    const bool& isValid() const { return m_valid; }
    unsigned nbVertices() const { return m_vertices.size(); }
    unsigned nbNodes() const { return m_nodes.size(); }
    unsigned nbCurves() const { return m_curves.size(); }
    unsigned nbTriangles() const { return m_triangles.size(); }
    unsigned nbSingularTriangles() const { return m_singularTriangles.size(); }

public:
    const bool& getValid() const { return m_valid; }
    bool& getValid() { return m_valid; }

    const VertexList& getVertices() const { return m_vertices; }
    VertexList& getVertices() { return m_vertices; }

    const NodeList& getNodes() const { return m_nodes; }
    NodeList& getNodes() { return m_nodes; }

    const CurveList& getCurves() const { return m_curves; }
    CurveList& getCurves() { return m_curves; }

    const TriangleList& getTriangles() const { return m_triangles; }
    TriangleList& getTriangles() { return m_triangles; }

    const SingularTriangleList& getSingularTriangles() const { return m_singularTriangles; }
    SingularTriangleList& getSingularTriangles() { return m_singularTriangles; }

    const Eigen::AlignedBox2f& getBoundingBox() const { return m_boundingBox; }
    Eigen::AlignedBox2f& getBoundingBox() { return m_boundingBox; }

public:
    void clear();

protected:
    bool m_valid;

    VertexList m_vertices;
    NodeList m_nodes;
    CurveList m_curves;
    TriangleList m_triangles;
    SingularTriangleList m_singularTriangles;

    Eigen::AlignedBox2f m_boundingBox;
};


#include "qMesh.hpp"

} // namespace Vitelotte

#endif
