/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _EXAMPLES_VITELOTTE_COMMON_VG_NODE_RENDERER_
#define _EXAMPLES_VITELOTTE_COMMON_VG_NODE_RENDERER_


#include <Patate/vitelotte.h>
#include <Patate/vitelotte_gl.h>

//#include "vgMeshWithCurves.h"
#include "glPointRenderer.h"
#include "glLineRenderer.h"


class VGNodeRenderer
{
public:
//    typedef VGMeshWithCurves Mesh;
    typedef Vitelotte::VGMesh<float, Vitelotte::Dynamic, Vitelotte::Dynamic> Mesh;

    struct PlacedNode
    {
        Mesh::Node n;
        Mesh::Vector p;
    };

    typedef std::vector<PlacedNode> PlacedNodeList;

public:
    VGNodeRenderer();

    bool linearizeSrgb() const;
    void setLinearizeSrgb(bool linearizeSrgb);

    float nodeOffset() const;
    void setNodeOffset(float nodeOffset);

    float nodeRadius() const;
    void setNodeRadius(float nodeRadius);

    float edgeOffset() const;
    void setEdgeOffset(float edgeOffset);

    const Eigen::Vector4f& baseColor() const;
    void setBaseColor(const Eigen::Vector4f& baseColor);

    const Eigen::Vector4f& unconstrainedColor() const;
    void setUnconstrainedColor(const Eigen::Vector4f& unconstrainedColor);

    Mesh::Node highlightedNode() const;
    void setHighlightedNode(Mesh::Node n);

    Mesh::Node pickNode(const Mesh::Vector& p) const;

    void clear();
    void update(const Mesh& mesh, float zoom);

    void render(const Eigen::Matrix4f& transform,
                const Eigen::Vector2f& viewportSize);

private:
    bool fromSplit(const Mesh& mesh, Mesh::Halfedge h) const;
    bool isConstrained(const Mesh& mesh, Mesh::Halfedge h) const;

    void updateEdge(const Mesh& mesh, float zoom, Mesh::Edge e);
    void updateVertexNodes(const Mesh& mesh, float zoom, Mesh::Vertex vx);

    void addEdge(const Mesh::Vector& p0, const Mesh::Vector& p1,
                 bool constrained);
    void addNode(const Mesh& mesh, Mesh::Node n, const Mesh::Vector& p);
    void addNode2(const Mesh& mesh, Mesh::Node n0, Mesh::Node n1,
                  const Mesh::Vector& p, const Mesh::Vector& offset);

    Eigen::Vector4f convColor(const Eigen::Vector4f& color) const;

private:
    bool m_linearizeSrgb;
    float m_nodeOffset;
    float m_nodeRadius;
    float m_edgeOffset;
    Eigen::Vector4f m_baseColor;
    Eigen::Vector4f m_unconstrainedColor;

    Mesh::Node m_highlightedNode;
    PlacedNodeList m_placedNodes;

    GLPointRenderer m_pointRenderer;
    GLLineRenderer m_lineRenderer;
};


#endif
