/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <GL/glew.h>

#include "vgNodeRenderer.h"


VGNodeRenderer::VGNodeRenderer()
    : m_linearizeSrgb(true),
      m_nodeOffset(6),
      m_nodeRadius(4),
      m_edgeOffset(1.5),
      m_baseColor(0, 0, 0, 1),
//      m_unconstrainedColor(.5, .5, .5, 1)
    m_unconstrainedColor(0, 0, 0, .5)
{
}


bool VGNodeRenderer::linearizeSrgb() const
{
    return m_linearizeSrgb;
}


void VGNodeRenderer::setLinearizeSrgb(bool linearizeSrgb)
{
    m_linearizeSrgb = linearizeSrgb;
}


float VGNodeRenderer::nodeOffset() const
{
    return m_nodeOffset;
}


void VGNodeRenderer::setNodeOffset(float nodeOffset)
{
    m_nodeOffset = nodeOffset;
}


float VGNodeRenderer::nodeRadius() const
{
    return m_nodeRadius;
}


void VGNodeRenderer::setNodeRadius(float nodeRadius)
{
    m_nodeRadius = nodeRadius;
}


float VGNodeRenderer::edgeOffset() const
{
    return m_edgeOffset;
}


void VGNodeRenderer::setEdgeOffset(float edgeOffset)
{
    m_edgeOffset = edgeOffset;
}


const Eigen::Vector4f& VGNodeRenderer::baseColor() const
{
    return m_baseColor;
}


void VGNodeRenderer::setBaseColor(const Eigen::Vector4f& baseColor)
{
    m_baseColor = baseColor;
}


const Eigen::Vector4f& VGNodeRenderer::unconstrainedColor() const
{
    return m_unconstrainedColor;
}


void VGNodeRenderer::setUnconstrainedColor(const Eigen::Vector4f& unconstrainedColor)
{
    m_unconstrainedColor = unconstrainedColor;
}


VGNodeRenderer::Mesh::Node VGNodeRenderer::highlightedNode() const
{
    return m_highlightedNode;
}


void VGNodeRenderer::setHighlightedNode(Mesh::Node n)
{
    m_highlightedNode = n;
}


VGNodeRenderer::Mesh::Node VGNodeRenderer::pickNode(const Mesh::Vector& p) const
{
    if(m_placedNodes.empty())
    {
        return Mesh::Node();
    }

    Mesh::Node n = m_placedNodes.front().n;
    float minSqrDist = (m_placedNodes.front().p - p).squaredNorm();
    for(PlacedNodeList::const_iterator pnit = m_placedNodes.begin();
        pnit != m_placedNodes.end(); ++pnit)
    {
        float sqrDist = (pnit->p - p).squaredNorm();
        if(sqrDist < minSqrDist)
        {
            n = pnit->n;
            minSqrDist = sqrDist;
        }
    }
    return n;
}


void VGNodeRenderer::clear()
{
    m_placedNodes.clear();
    m_pointRenderer.clear();
    m_lineRenderer.clear();
}


// TODO: move this
float angle(const Eigen::Vector2f& v0, const Eigen::Vector2f v1)
{
    // Vectors must be normalized
    return std::atan2(Vitelotte::det2(v0, v1), v0.dot(v1));
}


void VGNodeRenderer::update(const Mesh& mesh, float zoom)
{
    // TODO: support mesh with more than 2 dimesions.
    assert(mesh.nDims() == 2);

    clear();

    for(Mesh::EdgeIterator eit = mesh.edgesBegin();
        eit != mesh.edgesEnd(); ++eit)
    {
        updateEdge(mesh, zoom, *eit);
    }


    for(Mesh::VertexIterator vit = mesh.verticesBegin();
        vit != mesh.verticesEnd(); ++vit)
    {
        updateVertexNodes(mesh, zoom, *vit);
    }
}


void VGNodeRenderer::render(const Eigen::Matrix4f& transform,
                            const Eigen::Vector2f& viewportSize)
{
    m_lineRenderer.upload();
    m_lineRenderer.render(transform, viewportSize);

    m_pointRenderer.upload();
    m_pointRenderer.render(transform, viewportSize);
}


bool VGNodeRenderer::fromSplit(const Mesh& mesh, Mesh::Halfedge h) const
{
    Mesh::Halfedge oh = mesh.oppositeHalfedge(h);
    if(mesh.isBoundary(h) || mesh.isBoundary(oh))
        return false;
    return mesh.fromVertexValueNode(h) != mesh.toVertexValueNode(oh);
}


bool VGNodeRenderer::isConstrained(const Mesh& mesh, Mesh::Halfedge h) const
{
    Mesh::Halfedge oh = mesh.oppositeHalfedge(h);
    return   mesh.isConstraint(mesh.fromVertexValueNode(h))
        ||  (mesh.hasEdgeValue()
          && mesh.isConstraint(mesh.edgeValueNode(h)))
        ||  (mesh.hasEdgeGradient()
          && mesh.isConstraint(mesh.edgeGradientNode(h)))
        ||   mesh.isConstraint(mesh.toVertexValueNode(h))
        ||   mesh.fromVertexValueNode(h) != mesh.fromVertexValueNode(oh)
        ||  (mesh.hasEdgeValue()
          && mesh.edgeValueNode(h)       != mesh.edgeValueNode(oh))
        ||  (mesh.hasEdgeGradient()
          && mesh.edgeGradientNode(h)    != mesh.edgeGradientNode(oh))
        ||   mesh.toVertexValueNode(h)   != mesh.toVertexValueNode(oh);
}


void VGNodeRenderer::updateEdge(const Mesh& mesh, float zoom, Mesh::Edge e)
{
    Mesh::Halfedge h = mesh.halfedge(e, 0);
    Mesh::Halfedge oh = mesh.halfedge(e, 1);

    Mesh::Vector p0 = mesh.position(mesh.fromVertex(h));
    Mesh::Vector p1 = mesh.position(mesh.toVertex(h));
    Mesh::Vector mid = (p0 + p1) / 2;

    Mesh::Vector v = (p1 - p0).normalized();
    Mesh::Vector n(2); n << -v.y(), v.x();

    bool boundary = mesh.isBoundary(e);
    bool fromSplit = !boundary && mesh.fromVertexValueNode(h) != mesh.toVertexValueNode(oh);
    bool midSplit  = !boundary && mesh.hasEdgeValue()
                               && mesh.edgeValueNode(h)       != mesh.edgeValueNode(oh);
    bool toSplit   = !boundary && mesh.toVertexValueNode(h)   != mesh.fromVertexValueNode(oh);

    bool constrained = isConstrained(mesh, h);

    float offset = m_edgeOffset / zoom;

    if(!fromSplit && !midSplit && !toSplit)
    {
        addEdge(p0, p1, constrained);
    }
    else
    {
        addEdge(p0 + n*offset, p1 + n*offset, constrained);
        addEdge(p0 - n*offset, p1 - n*offset, constrained);
    }

    if(mesh.hasEdgeValue())
    {
        float noffset = m_nodeOffset / zoom;
        if((p1 - p0).squaredNorm() >= 64 * noffset * noffset)
        {
            addNode2(mesh, mesh.edgeValueNode(h), mesh.edgeValueNode(oh),
                     mid, n*noffset);
        }
    }
}


void VGNodeRenderer::updateVertexNodes(const Mesh& mesh, float zoom, Mesh::Vertex vx)
{
    Mesh::HalfedgeAroundVertexCirculator hit = mesh.halfedges(vx);
    Mesh::HalfedgeAroundVertexCirculator hend = hit;

    Mesh::Vector p = mesh.position(vx);

    float radius = 0;
    float minSqrEdgeLen = (mesh.position(mesh.toVertex(*hit)) - p).squaredNorm();
    float offset = m_nodeOffset / zoom;
    bool constrained = false;
    do
    {
        if(mesh.isBoundary(*hit))
        {
            ++hit;
            continue;
        }

        int count = 2;
        count += fromSplit(mesh, *hit);
        Mesh::Vertex vx0 = mesh.fromVertex(*hit);
        Mesh::Vertex vx1 = mesh.toVertex(*hit);
        minSqrEdgeLen = std::min(minSqrEdgeLen,
                (mesh.position(vx1) - mesh.position(vx0)).squaredNorm());
        constrained |= isConstrained(mesh, *hit);

        ++hit;

        count += fromSplit(mesh, *hit);
        Mesh::Vertex vx2 = mesh.toVertex(*hit);

        Mesh::Vector v1 = (mesh.position(vx1) - mesh.position(vx0)).normalized();
        Mesh::Vector v2 = (mesh.position(vx2) - mesh.position(vx0)).normalized();
        radius = std::max(radius, count * offset / angle(v1, v2));
    } while(hit != hend);

    if(16*radius*radius < minSqrEdgeLen)
    {
        do
        {
            Mesh::Vertex vx0 = mesh.fromVertex(*hit);
            Mesh::Vertex vx1 = mesh.toVertex(*hit);

            Mesh::Vector v = (mesh.position(vx1) - mesh.position(vx0)).normalized();
            Mesh::Vector n(2); n << -v.y(), v.x();

            addNode2(mesh, mesh.fromVertexValueNode(*hit),
                     mesh.toVertexValueNode(mesh.oppositeHalfedge(*hit)),
                     p + v*radius, n*offset);

            ++hit;
        } while(hit != hend);
    }

    if(constrained)
    {
        m_pointRenderer.addPoint((Eigen::Vector3f() << p, 0).finished(),
                                 m_edgeOffset + 1.5, convColor(m_baseColor));
    }
}


void VGNodeRenderer::addEdge(const Mesh::Vector& p0, const Mesh::Vector& p1,
                             bool constrained)
{
    Eigen::Vector4f c = convColor(constrained? m_baseColor: m_unconstrainedColor);
    float r = constrained? 1.5: .5;

    m_lineRenderer.addPoint((Eigen::Vector3f() << p0, 0).finished(),
                            r, c);
    m_lineRenderer.addPoint((Eigen::Vector3f() << p1, 0).finished(),
                            r, c);
    m_lineRenderer.endLine();
}


void VGNodeRenderer::addNode(const Mesh& mesh, Mesh::Node n, const Mesh::Vector& p){
    Eigen::Vector4f color = convColor(
                (mesh.isValid(n) && n == m_highlightedNode)?
                    Eigen::Vector4f(1, 1, 1, 1):
                    m_baseColor);

    if(mesh.isValid(n))
    {
        PlacedNode pn;
        pn.n = n;
        pn.p = p;
        m_placedNodes.push_back(pn);
    }

    if(mesh.isValid(n) && mesh.isConstraint(n))
    {
        m_pointRenderer.addPoint((Eigen::Vector3f() << p, 0).finished(),
                                 m_nodeRadius + .5, color);

        Mesh::Value v = mesh.value(n);
        v(3) = 1;
        m_pointRenderer.addPoint((Eigen::Vector3f() << p, 0).finished(),
                                 m_nodeRadius - 1, convColor(v));
    }
    else
    {
        m_pointRenderer.addPoint((Eigen::Vector3f() << p, 0).finished(),
                                 1.5, color);
    }
}


void VGNodeRenderer::addNode2(const Mesh& mesh, Mesh::Node n0, Mesh::Node n1,
                              const Mesh::Vector& p, const Mesh::Vector& offset)
{
    if(n0 == n1 || !mesh.isValid(n0) || !mesh.isValid(n1))
    {
        Mesh::Node node = mesh.isValid(n0)? n0: n1;
        addNode(mesh, node, p);
    }
    else
    {
        addNode(mesh, n0, p + offset);
        addNode(mesh, n1, p - offset);
    }
}


Eigen::Vector4f VGNodeRenderer::convColor(const Eigen::Vector4f& color) const
{
    if(m_linearizeSrgb)
    {
        return PatateCommon::srgbToLinear(color);
    }
    return color;
}
