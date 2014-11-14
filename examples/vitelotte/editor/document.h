#ifndef _DOCUMENT_H_
#define _DOCUMENT_H_


#include <Eigen/Geometry>

#include <QObject>
#include <QUndoCommand>
#include <QUndoStack>

#include "Patate/vitelotte.h"


class Document : public QObject
{
    Q_OBJECT

public:
    typedef Vitelotte::VGMesh<float> Mesh;
    typedef Vitelotte::FVElementBuilder<Mesh> FVElement;
    typedef Vitelotte::FemSolver<Mesh, FVElement> FVSolver;
    typedef Eigen::AlignedBox2f BoundingBox;

    enum HalfedgeNode
    {
        FromValueNode,
        ToValueNode,
        MaxVertexNode,  // node can be swapped by inverting the first bit if
                        // it is a vertex node (n < MaxVertexNode).

        EdgeValueNode = MaxVertexNode,
        EdgeGradientNode,

        MaxHalfedgeNode = 0xff  // Ensure the enum is big enough
    };


public:
    Document(QObject* parent = 0);

    const BoundingBox& boundingBox() const;
    void updateBoundingBox();

    Mesh::Edge selectedEdge() const;
    void setSelectedEdge(Mesh::Edge e);

    float edgeSqrDist(Mesh::Edge e, const Eigen::Vector2f& p) const;
    Mesh::Edge closestEdge(const Eigen::Vector2f& p) const;

    void solve();

    Mesh& mesh();
    const Mesh& mesh() const;

    Mesh& solvedMesh();
    const Mesh& solvedMesh() const;

    HalfedgeNode swapNode(HalfedgeNode nid) const;
    Mesh::Node meshNode(Mesh::Halfedge h, HalfedgeNode nid) const;
    Mesh::Node& meshNode(Mesh::Halfedge h, HalfedgeNode nid);


    void splitNode(Mesh::Halfedge h, HalfedgeNode nid);
    void mergeNode(Mesh::Halfedge h, HalfedgeNode nid);
    void setNodeValue(Mesh::Halfedge h, HalfedgeNode nid,
                      const Mesh::NodeValue& value);


    QUndoStack* undoStack();

public slots:
    void loadMesh(const std::string& filename);

    void openLoadMeshDialog();
    void openSaveSourceMeshDialog();
    void openSaveFinalMeshDialog();


signals:
    void selectedEdgeChanged();
    void meshUpdated();


private:
    Mesh m_mesh;
    Mesh m_solvedMesh;
    BoundingBox m_bb;

    Mesh::Edge m_selectedEdge;

    FVSolver m_fvSolver;

    QUndoStack* m_undoStack;
};


class SetNodeValue : public QUndoCommand
{
public:
    typedef Document::Mesh::Node Node;
    typedef Document::Mesh::NodeValue NodeValue;

public:
    SetNodeValue(Document* doc, Node node, const NodeValue& value);

    virtual void undo();
    virtual void redo();

private:
    Document* m_document;
    Node m_node;
    NodeValue m_newValue;
    NodeValue m_prevValue;
};


class SetNode : public QUndoCommand
{
public:
    typedef Document::Mesh::Halfedge Halfedge;
    typedef Document::Mesh::Node Node;

public:
    SetNode(Document* doc, Halfedge halfedge, Document::HalfedgeNode nid,
            Node node);

    virtual void undo();
    virtual void redo();

private:
    Document* m_document;
    Halfedge m_halfedge;
    Document::HalfedgeNode m_nid;
    Node m_newNode;
    Node m_prevNode;
};


#endif
