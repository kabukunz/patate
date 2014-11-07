#include "Patate/vitelotte_io.h"

#include "document.h"


Document::Document()
    : m_fvSolver(&m_solvedMesh)
{

}


const Document::BoundingBox& Document::boundingBox() const
{
    return m_bb;
}


void Document::updateBoundingBox()
{
    typedef Mesh::VertexIterator VertexIterator;

    m_bb.setEmpty();
    for(VertexIterator vit = m_mesh.verticesBegin();
        vit != m_mesh.verticesEnd(); ++vit)
        m_bb.extend(m_mesh.position(*vit));
}


Document::Mesh::Edge Document::selectedEdge() const
{
    return m_selectedEdge;
}


void Document::setSelectedEdge(Mesh::Edge e)
{
    assert(!e.isValid() || m_mesh.isValid(e));
    if(e != m_selectedEdge)
    {
        m_selectedEdge = e;
        emit selectedEdgeChanged();
    }
}


float Document::edgeSqrDist(Mesh::Edge e, const Eigen::Vector2f& p) const
{
    Eigen::Vector2f p0 = m_mesh.position(m_mesh.vertex(e, 0));
    Eigen::Vector2f p1 = m_mesh.position(m_mesh.vertex(e, 1));

    Eigen::Vector2f edge = p1 - p0;

    Eigen::Vector2f v0 = p - p0;
    float d0 = edge.dot(v0);
    if(d0 <= 0)
        return v0.squaredNorm();

    Eigen::Vector2f v1 = p - p1;
    if(edge.dot(v1) >= 0)
        return v1.squaredNorm();

    return v0.squaredNorm() - d0*d0 / edge.squaredNorm();
}


Document::Mesh::Edge Document::closestEdge(const Eigen::Vector2f& p) const
{
    Mesh::EdgeIterator eit = m_mesh.edgesBegin();
    Mesh::Edge closest = *eit;
    float dist = edgeSqrDist(*eit, p);

    for(++eit; eit != m_mesh.edgesEnd(); ++eit)
    {
        float edist = edgeSqrDist(*eit, p);
        if(edist < dist)
        {
            dist = edist;
            closest = *eit;
        }
    }

    return closest;
}


void Document::solve()
{
    m_solvedMesh = m_mesh;
    m_solvedMesh.finalize();
    m_solvedMesh.compactNodes();

    m_fvSolver.build();
    m_fvSolver.sort();
    m_fvSolver.solve();

    emit meshUpdated();
}


void Document::loadMesh(const std::string& filename)
{
    Vitelotte::readMvgFromFile(filename, m_mesh);
    m_mesh.setAttributes(Mesh::FV);

    updateBoundingBox();
    setSelectedEdge(Mesh::Edge());

    solve();
}


Document::Mesh& Document::mesh()
{
    return m_mesh;
}


const Document::Mesh& Document::mesh() const
{
    return m_mesh;
}


Document::Mesh& Document::solvedMesh()
{
    return m_solvedMesh;
}


const Document::Mesh& Document::solvedMesh() const
{
    return m_solvedMesh;
}
