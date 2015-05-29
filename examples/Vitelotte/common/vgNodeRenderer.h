/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _EXAMPLES_VITELOTTE_COMMON_VG_NODE_RENDERER_
#define _EXAMPLES_VITELOTTE_COMMON_VG_NODE_RENDERER_


#include <vector>

#include <Eigen/Geometry>

#include <Patate/vitelotte.h>
#include <Patate/vitelotte_gl.h>

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
        Eigen::Vector2f p;
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

    Mesh::Node pickNode(const Eigen::Vector2f& p) const;

    void clear();
    void update(const Mesh& mesh,
                const Eigen::Matrix4f& transform,
                const Eigen::Vector2f& viewportSize);

    void render();

private:
    bool isFromSplit(const Mesh& mesh, Mesh::Halfedge h) const;
    bool isConstrained(const Mesh& mesh, Mesh::Halfedge h) const;
    Eigen::Vector2f project(const Mesh::Vector& pos) const;

    Eigen::Vector2f projPos(Mesh::Vertex vx) const;

    void updateEdge(const Mesh& mesh, Mesh::Edge e);
    void updateVertexNodes(const Mesh& mesh, Mesh::Vertex vx);

    void addEdge(const Eigen::Vector2f& p0, const Eigen::Vector2f& p1,
                 bool constrained);
    void addNode(const Mesh& mesh, Mesh::Node n, const Eigen::Vector2f& p);
    void addNode2(const Mesh& mesh, Mesh::Node n0, Mesh::Node n1,
                  const Eigen::Vector2f& p, const Eigen::Vector2f& offset);

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

    std::vector<Eigen::Vector2f> m_projPos;

    Eigen::Matrix4f m_transform;
    Eigen::Vector2f m_viewportSize;
    Eigen::AlignedBox2f m_viewBox;
};


#endif
