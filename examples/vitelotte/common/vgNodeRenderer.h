#ifndef _PATATE_EXAMPLES_COMMON_VG_NODE_RENDERER_H
#define _PATATE_EXAMPLES_COMMON_VG_NODE_RENDERER_H


#include <Patate/vitelotte.h>

#include "glPointRenderer.h"
#include "glLineRenderer.h"


class VGNodeRenderer
{
public:
    typedef Vitelotte::VGMesh<float> Mesh;

public:
    VGNodeRenderer();

    void clear();
    void update(const Mesh& mesh, float zoom);

    void render(const Eigen::Matrix4f& transform,
                const Eigen::Vector2f& viewportSize);

private:
    bool fromSplit(const Mesh& mesh, Mesh::Halfedge h) const;

    void updateEdge(const Mesh& mesh, float zoom, Mesh::Edge e);
    void updateVertexNodes(const Mesh& mesh, float zoom, Mesh::Vertex vx);

    void addEdge(const Mesh::Vector& p0, const Mesh::Vector& p1);
    void addNode(const Mesh& mesh, Mesh::Node n, const Mesh::Vector& p);

private:
    float m_nodeOffset;
    float m_nodeRadius;
    float m_edgeOffset;

    GLPointRenderer m_pointRenderer;
    GLLineRenderer m_lineRenderer;
};


#endif
