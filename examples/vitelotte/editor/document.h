#ifndef _DOCUMENT_H_
#define _DOCUMENT_H_


#include <Eigen/Geometry>

#include <QObject>
#include <QUndoCommand>
#include <QUndoStack>

#include "Patate/vitelotte.h"

#include "../common/vgMeshWithCurves.h"


typedef VGMeshWithCurves Mesh;


class MeshSelection
{
public:
    enum SelectionType
    {
        SelectionNone,
        SelectionVertex,
        SelectionEdge
    };

public:
    MeshSelection();
    MeshSelection(Mesh::Vertex v);
    MeshSelection(Mesh::Edge e);

    bool operator==(const MeshSelection& other) const;
    bool operator!=(const MeshSelection& other) const;

    SelectionType type() const;
    bool isNone() const;
    bool isVertex() const;
    bool isEdge() const;

    Mesh::Vertex vertex() const;
    Mesh::Edge edge() const;

private:
    SelectionType m_type;
    int m_index;
};


class Document : public QObject
{
    Q_OBJECT

public:
    typedef ::Mesh Mesh;
    typedef Vitelotte::FVElementBuilder<Mesh, double> FVElement;
    typedef Vitelotte::SingularElementDecorator<FVElement> Element;
    typedef Vitelotte::FemSolver<Mesh, Element> FVSolver;
    typedef Eigen::AlignedBox2f BoundingBox;

    enum MeshType
    {
        BASE_MESH,
        FINALIZED_MESH,
        SOLVED_MESH
    };


public:
    Document(QObject* parent = 0);

    const BoundingBox& boundingBox() const;
    void updateBoundingBox();

    MeshSelection selection() const;
    void setSelection(MeshSelection selection);
    bool isSelectionValid(MeshSelection selection);

    float vertexSqrDist(Mesh::Vertex v, const Eigen::Vector2f& p) const;
    Mesh::Vertex closestVertex(const Eigen::Vector2f& p, float* sqrDist=0) const;

    float edgeSqrDist(Mesh::Edge e, const Eigen::Vector2f& p) const;
    Mesh::Edge closestEdge(const Eigen::Vector2f& p, float* sqrDist=0) const;

    void solve();

    Mesh& getMesh(MeshType type);
    const Mesh& getMesh(MeshType type) const;

    Mesh& mesh();
    const Mesh& mesh() const;

    Mesh& finalizedMesh();
    const Mesh& finalizedMesh() const;

    Mesh& solvedMesh();
    const Mesh& solvedMesh() const;


    void splitNode(Mesh::Halfedge h, Mesh::HalfedgeAttribute nid);
    void mergeNode(Mesh::Halfedge h, Mesh::HalfedgeAttribute nid);
    void setNodeValue(Mesh::Halfedge h, Mesh::HalfedgeAttribute nid,
                      const Mesh::NodeValue& value, bool allowMerge=false);


    QUndoStack* undoStack();

public slots:
    void loadMesh(const std::string& filename);

    void openLoadMeshDialog();
    void openSaveSourceMeshDialog();
    void openSaveFinalMeshDialog();

    void exportPlot(const std::string& filename, unsigned layer=3);

signals:
    void selectionChanged();
    void meshChanged();
    void meshUpdated();


private:
    Mesh m_mesh;
    Mesh m_finalizedMesh;
    Mesh m_solvedMesh;
    BoundingBox m_bb;

    MeshSelection m_selection;

    FVSolver m_fvSolver;

    QUndoStack* m_undoStack;
};


class SetNodeValue : public QUndoCommand
{
public:
    typedef Document::Mesh::Node Node;
    typedef Document::Mesh::NodeValue NodeValue;

public:
    SetNodeValue(Document* doc, Node node, const NodeValue& value,
                 bool allowMerge=false);

    virtual void undo();
    virtual void redo();

    virtual int id() const;
    virtual bool mergeWith(const QUndoCommand* command);

private:
    Document* m_document;
    Node m_node;
    NodeValue m_newValue;
    NodeValue m_prevValue;
    bool m_allowMerge;
};


class SetNode : public QUndoCommand
{
public:
    typedef Document::Mesh::Halfedge Halfedge;
    typedef Document::Mesh::Node Node;

public:
    SetNode(Document* doc, Halfedge halfedge, Mesh::HalfedgeAttribute nid,
            Node node);

    virtual void undo();
    virtual void redo();

private:
    Document* m_document;
    Halfedge m_halfedge;
    Mesh::HalfedgeAttribute m_nid;
    Node m_newNode;
    Node m_prevNode;
};


#endif
