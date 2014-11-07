#ifndef _DOCUMENT_H_
#define _DOCUMENT_H_


#include <Eigen/Geometry>

#include <QObject>

#include "Patate/vitelotte.h"


class Document : public QObject
{
    Q_OBJECT

public:
    typedef Vitelotte::VGMesh<float> Mesh;
    typedef Vitelotte::FVElementBuilder<Mesh> FVElement;
    typedef Vitelotte::FemSolver<Mesh, FVElement> FVSolver;
    typedef Eigen::AlignedBox2f BoundingBox;


public:
    Document();

    const BoundingBox& boundingBox() const;
    void updateBoundingBox();

    Mesh::Edge selectedEdge() const;
    void setSelectedEdge(Mesh::Edge e);

    float edgeSqrDist(Mesh::Edge e, const Eigen::Vector2f& p) const;
    Mesh::Edge closestEdge(const Eigen::Vector2f& p) const;

    void solve();

public slots:
    void loadMesh(const std::string& filename);

    Mesh& mesh();
    const Mesh& mesh() const;

    Mesh& solvedMesh();
    const Mesh& solvedMesh() const;


signals:
    void selectedEdgeChanged();
    void meshUpdated();


private:
    Mesh m_mesh;
    Mesh m_solvedMesh;
    BoundingBox m_bb;

    Mesh::Edge m_selectedEdge;

    FVSolver m_fvSolver;
};


#endif
