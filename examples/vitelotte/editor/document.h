#ifndef _DOCUMENT_H_
#define _DOCUMENT_H_


#include <Eigen/Geometry>

#include <QObject>
#include <QUndoCommand>
#include <QUndoStack>

#include "Patate/vitelotte.h"

#include "../common/vgMeshWithCurves.h"


typedef float Scalar;
typedef VGMeshWithCurves Mesh;


class MeshSelection
{
public:
    enum SelectionType
    {
        SelectionNone,
        SelectionVertex,
        SelectionEdge,
        SelectionPointConstraint,
        SelectionCurve
    };

public:
    MeshSelection();
    MeshSelection(Mesh::Vertex v);
    MeshSelection(Mesh::Edge e);
    MeshSelection(Mesh::PointConstraint pc);
    MeshSelection(Mesh::Curve c);

    bool operator==(const MeshSelection& other) const;
    bool operator!=(const MeshSelection& other) const;

    SelectionType type() const;
    bool isNone() const;
    bool isVertex() const;
    bool isEdge() const;
    bool isPointConstraint() const;
    bool isCurve() const;

    Mesh::Vertex vertex() const;
    Mesh::Edge edge() const;
    Mesh::PointConstraint pointConstraint() const;
    Mesh::Curve curve() const;

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

    enum
    {
        CLEAN              = 0,
        DIRTY_NODE_VALUE   = 1,
        DIRTY_NODE_TYPE    = 2,
        DIRTY_CONNECTIVITY = 3,

        DIRTY_LEVEL_MASK   = 0x0f,
        DIRTY_CURVES_FLAG  = 0x10
    };

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
    Mesh::PointConstraint closestPointConstraint(const Eigen::Vector2f& p, float* sqrDist=0) const;

    float edgeSqrDist(Mesh::Edge e, const Eigen::Vector2f& p) const;
    Mesh::Edge closestEdge(const Eigen::Vector2f& p, float* sqrDist=0) const;
    Mesh::Curve closestCurve(const Eigen::Vector2f& p, float* sqrDist=0) const;

    void markDirty(unsigned flags);
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

    void exportPlot(const std::string& filename, unsigned layer=0);

signals:
    void selectionChanged();
    void meshChanged();
    void meshUpdated();


private:
    bool connectivityChanged() const;

private:
    Mesh m_mesh;
    Mesh m_finalizedMesh;
    Mesh m_solvedMesh;
    BoundingBox m_bb;

    MeshSelection m_selection;

    unsigned m_dirtyFlags;
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


class MoveGradientStop : public QUndoCommand
{
public:
    typedef Document::Mesh::Curve Curve;

public:
    MoveGradientStop(Document* doc, Curve curve, unsigned which, Scalar fromPos, Scalar toPos,
            bool allowMerge=false);

    virtual void undo();
    virtual void redo();

    virtual int id() const;
    virtual bool mergeWith(const QUndoCommand* command);

private:
    void move(Scalar from, Scalar to);

private:
    Document* m_document;
    Curve m_curve;
    unsigned m_which;
    Scalar m_fromPos;
    Scalar m_toPos;
    bool m_allowMerge;
};


class SetGradientStopValue : public QUndoCommand
{
public:
    typedef Document::Mesh::Curve Curve;
    typedef Document::Mesh::NodeValue NodeValue;

public:
    SetGradientStopValue(Document* doc, Curve curve, unsigned which,
                         Scalar pos, const NodeValue& value);

    virtual void undo();
    virtual void redo();

private:
    void setColor(const NodeValue& color);

private:
    Document* m_document;
    Curve m_curve;
    unsigned m_which;
    Scalar m_pos;
    NodeValue m_prevValue;
    NodeValue m_newValue;
};


class AddRemoveGradientStop : public QUndoCommand
{
public:
    typedef Document::Mesh::Curve Curve;
    typedef Document::Mesh::NodeValue NodeValue;

public:
    // Add
    AddRemoveGradientStop(Document* doc, Curve curve, unsigned which,
                          Scalar pos, const NodeValue& value);
    // Remove
    AddRemoveGradientStop(Document* doc, Curve curve, unsigned which,
                          Scalar pos);

    virtual void undo();
    virtual void redo();

private:
    void add();
    void remove();

private:
    Document* m_document;
    Curve m_curve;
    unsigned m_which;
    Scalar m_pos;
    NodeValue m_value;
    bool m_add;
};


#endif
