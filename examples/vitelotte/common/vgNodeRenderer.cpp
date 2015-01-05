#include <GL/glew.h>

#include "vgNodeRenderer.h"


VGNodeRenderer::VGNodeRenderer()
    : m_nodeOffset(6),
      m_nodeRadius(4),
      m_edgeOffset(2) {
}

void VGNodeRenderer::clear()
{
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
    clear();

    float offset = m_nodeOffset / zoom;

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
    return mesh.vertexFromValueNode(h) != mesh.vertexValueNode(oh);
}


void VGNodeRenderer::updateEdge(const Mesh& mesh, float zoom, Mesh::Edge e)
{
    Mesh::Halfedge h = mesh.halfedge(e, 0);
    Mesh::Halfedge oh = mesh.halfedge(e, 1);

    Mesh::Vector p0 = mesh.position(mesh.fromVertex(h));
    Mesh::Vector p1 = mesh.position(mesh.toVertex(h));
    Mesh::Vector mid = (p0 + p1) / 2;

    Mesh::Vector v = (p1 - p0).normalized();
    Mesh::Vector n(-v.y(), v.x());

    bool boundary = mesh.isBoundary(e);
    bool fromSplit = !boundary && mesh.vertexFromValueNode(h) != mesh.vertexValueNode(oh);
    bool midSplit = !boundary && mesh.edgeValueNode(h) != mesh.edgeValueNode(oh);
    bool toSplit = !boundary && mesh.vertexValueNode(h) != mesh.vertexFromValueNode(oh);

    float offset = m_edgeOffset / zoom;

    if(!fromSplit && !midSplit && !toSplit)
    {
        addEdge(p0, p1);
    }
    else if(fromSplit && midSplit && toSplit)
    {
        addEdge(p0 + n*offset, p1 + n*offset);
        addEdge(p0 - n*offset, p1 - n*offset);
    }
    else
    {
        if(!fromSplit && !midSplit)
        {
            addEdge(p0, mid);
        }
        else
        {
            addEdge(p0 + n*offset*fromSplit, mid + n*offset*midSplit);
            addEdge(p0 - n*offset*fromSplit, mid - n*offset*midSplit);
        }

        if(!midSplit && !toSplit)
        {
            addEdge(mid, p1);
        }
        else
        {
            addEdge(mid + n*offset*midSplit, p1 + n*offset*toSplit);
            addEdge(mid - n*offset*midSplit, p1 - n*offset*toSplit);
        }
    }

    if(midSplit)
    {
        float noffset = m_nodeOffset / zoom;
        addNode(mesh, mesh.edgeValueNode(h),
                mid + n*noffset);
        addNode(mesh, mesh.edgeValueNode(oh),
                mid - n*noffset);
    }
    else
    {
        if(mesh.isValid(mesh.edgeValueNode(h)))
        {
            addNode(mesh, mesh.edgeValueNode(h),
                    mid);
        }
        else
        {
            addNode(mesh, mesh.edgeValueNode(mesh.oppositeHalfedge(oh)),
                    mid);
        }
    }
}


void VGNodeRenderer::updateVertexNodes(const Mesh& mesh, float zoom, Mesh::Vertex vx)
{
    Mesh::HalfedgeAroundVertexCirculator hit = mesh.halfedges(vx);
    Mesh::HalfedgeAroundVertexCirculator hend = hit;

    float radius = 0;
    //float minSqrEdgeLen = hit
    float offset = m_nodeOffset / zoom;
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

        ++hit;

        count += fromSplit(mesh, *hit);
        Mesh::Vertex vx2 = mesh.toVertex(*hit);

        Mesh::Vector v1 = (mesh.position(vx1) - mesh.position(vx0)).normalized();
        Mesh::Vector v2 = (mesh.position(vx2) - mesh.position(vx0)).normalized();
        radius = std::max(radius, count * offset / angle(v1, v2));
    } while(hit != hend);

    Mesh::Vector p = mesh.position(vx);
    do
    {
        Mesh::Vertex vx0 = mesh.fromVertex(*hit);
        Mesh::Vertex vx1 = mesh.toVertex(*hit);

        Mesh::Vector v = (mesh.position(vx1) - mesh.position(vx0)).normalized();
        Mesh::Vector n(-v.y(), v.x());

        if(fromSplit(mesh, *hit))
        {
            addNode(mesh, mesh.vertexFromValueNode(*hit),
                    p + v*radius + n*offset);
            addNode(mesh, mesh.vertexValueNode(mesh.oppositeHalfedge(*hit)),
                    p + v*radius - n*offset);
        }
        else
        {
            if(mesh.isValid(mesh.vertexFromValueNode(*hit)))
            {
                addNode(mesh, mesh.vertexFromValueNode(*hit),
                        p + v*radius);
            }
            else
            {
                addNode(mesh, mesh.vertexValueNode(mesh.oppositeHalfedge(*hit)),
                        p + v*radius);
            }
        }

        ++hit;
    } while(hit != hend);

    m_pointRenderer.addPoint((Eigen::Vector3f() << p, 0).finished(),
                             m_edgeOffset + 2, Eigen::Vector4f(0, 0, 0, 1));
}


void VGNodeRenderer::addEdge(const Mesh::Vector& p0, const Mesh::Vector& p1)
{
    m_lineRenderer.addPoint((Eigen::Vector3f() << p0, 0).finished(),
                            1.5, Eigen::Vector4f(0., 0., 0., 1.));
    m_lineRenderer.addPoint((Eigen::Vector3f() << p1, 0).finished(),
                            1.5, Eigen::Vector4f(0., 0., 0., 1.));
    m_lineRenderer.endLine();
}


void VGNodeRenderer::addNode(const Mesh& mesh, Mesh::Node n, const Mesh::Vector& p){
    if(mesh.isValid(n))
    {
        m_pointRenderer.addPoint((Eigen::Vector3f() << p, 0).finished(),
                                 m_nodeRadius + .5, Eigen::Vector4f(0, 0, 0, 1));
        m_pointRenderer.addPoint((Eigen::Vector3f() << p, 0).finished(),
                                 m_nodeRadius - 1, mesh.nodeValue(n));
    }
    else
    {
        m_pointRenderer.addPoint((Eigen::Vector3f() << p, 0).finished(),
                                 2, Eigen::Vector4f(0, 0, 0, 1));
    }
}
