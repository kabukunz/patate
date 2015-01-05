#include <GL/glew.h>

#include "vgNodeRenderer.h"


VGNodeRenderer::VGNodeRenderer()
{
}

void VGNodeRenderer::clear()
{
    m_pointRenderer.clear();
    m_lineRenderer.clear();
}


void VGNodeRenderer::update(const Mesh& mesh, float zoom)
{
    clear();

    for(Mesh::EdgeIterator eit = mesh.edgesBegin();
        eit != mesh.edgesEnd(); ++eit)
    {
        Mesh::Halfedge h = mesh.halfedge(*eit, 0);

        Mesh::Vector p0 = mesh.position(mesh.fromVertex(h));
        Mesh::Vector p1 = mesh.position(mesh.toVertex(h));
        Mesh::Vector p2 = mesh.position(mesh.toVertex(mesh.nextHalfedge(h)));

        m_lineRenderer.addPoint((Eigen::Vector3f() << p0, 0).finished(),
                                4, Eigen::Vector4f(1., 0., 0., 1.));
        m_lineRenderer.addPoint((Eigen::Vector3f() << p1, 0).finished(),
                                4, Eigen::Vector4f(1., 0., 0., 1.));
        m_lineRenderer.endLine();

        Mesh::Vector v1 = (p1 - p0).normalized();
        Mesh::Vector v2 = (p2 - p0).normalized();


        float offset = 8 / zoom;
        std::cout << offset << "\n";

        Eigen::Matrix2f m;
        m << -v1[1], v1[0], -v2[1], v2[0];
        Mesh::Vector v = m.partialPivLu().solve(Mesh::Vector(offset, -2 * offset));
        Mesh::Vector n0 = p0 + v;

        m_pointRenderer.addPoint((Eigen::Vector3f() << n0, 0).finished(),
                                 8, Eigen::Vector4f(0, 1, 0, 1));

        m << -v1[1], v1[0], -v2[1], v2[0];
        v = m.partialPivLu().solve(Mesh::Vector(2 * offset, -offset));
        Mesh::Vector n1 = p0 + v;

        m_pointRenderer.addPoint((Eigen::Vector3f() << n1, 0).finished(),
                                 8, Eigen::Vector4f(0, 1, 0, 1));
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
