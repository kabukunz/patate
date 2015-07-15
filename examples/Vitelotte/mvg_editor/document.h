/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _MVG_EDITOR_DOCUMENT_
#define _MVG_EDITOR_DOCUMENT_


#include <Eigen/Geometry>

#include <QObject>
#include <QUndoCommand>
#include <QUndoStack>

#include "Patate/vitelotte.h"


typedef float Scalar;
typedef Vitelotte::DCMesh<
            float,
            Vitelotte::Dynamic,
            Vitelotte::Dynamic> Mesh;


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
        DIRTY_ALL          = 2,

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

    Mesh& getMesh(MeshType type);
    const Mesh& getMesh(MeshType type) const;

    Mesh& mesh();
    const Mesh& mesh() const;

    Mesh& finalizedMesh();
    const Mesh& finalizedMesh() const;

    Mesh& solvedMesh();
    const Mesh& solvedMesh() const;

    QUndoStack* undoStack();

    void splitNode(Mesh::Halfedge h, Mesh::HalfedgeAttribute nid);
    void mergeNode(Mesh::Halfedge h, Mesh::HalfedgeAttribute nid);
    void setNodeValue(Mesh::Halfedge h, Mesh::HalfedgeAttribute nid,
                      const Mesh::Value& value, bool allowMerge=false);

    void moveGradientStop(Mesh::Curve curve, unsigned which,
                          Scalar fromPos, Scalar toPos, bool allowMerge=false);
    void setGradientStopValue(Mesh::Curve curve, unsigned which,
                              Scalar pos, const Mesh::Value& value);
    void addGradientStop(Mesh::Curve curve, unsigned which,
                         Scalar pos, const Mesh::Value& value);
    void removeGradientStop(Mesh::Curve curve, unsigned which, Scalar pos);
    void setGradient(Mesh::Curve curve, unsigned which,
                     const Mesh::ValueFunction& gradient);
    void setCurveFlags(Mesh::Curve curve, unsigned flags);

    void setPointConstraintValue(Mesh::PointConstraint pc,
                                 const Mesh::Value& value);
    void setPointConstraintGradient(Mesh::PointConstraint pc,
                                    const Mesh::Gradient& gradient);

public slots:
    void solve();

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


// Low-level commands /////////////////////////////////////////////////////////


class SetNodeValue : public QUndoCommand
{
public:
    typedef Document::Mesh::Node Node;
    typedef Document::Mesh::Value Value;

public:
    SetNodeValue(Document* doc, Node node, const Value& value,
                 bool allowMerge=false);

    virtual void undo();
    virtual void redo();

    virtual int id() const;
    virtual bool mergeWith(const QUndoCommand* command);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    Document* m_document;
    Node m_node;
    Value m_newValue;
    Value m_prevValue;
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

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    Document* m_document;
    Halfedge m_halfedge;
    Mesh::HalfedgeAttribute m_nid;
    Node m_newNode;
    Node m_prevNode;
};


// High-level commands /////////////////////////////////////////////////////////


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
    typedef Document::Mesh::Value Value;

public:
    SetGradientStopValue(Document* doc, Curve curve, unsigned which,
                         Scalar pos, const Value& value);

    virtual void undo();
    virtual void redo();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    void setColor(const Value& color);

private:
    Document* m_document;
    Curve m_curve;
    unsigned m_which;
    Scalar m_pos;
    Value m_prevValue;
    Value m_newValue;
};


class AddRemoveGradientStop : public QUndoCommand
{
public:
    typedef Document::Mesh::Curve Curve;
    typedef Document::Mesh::Value Value;
    typedef Document::Mesh::ValueFunction ValueFunction;

public:
    // Add
    AddRemoveGradientStop(Document* doc, Curve curve, unsigned which,
                          Scalar pos, const Value& value);
    // Remove
    AddRemoveGradientStop(Document* doc, Curve curve, unsigned which,
                          Scalar pos);

    virtual void undo();
    virtual void redo();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    void add();
    void remove();

private:
    Document* m_document;
    Curve m_curve;
    unsigned m_which;
    Scalar m_pos;
    Value m_value;
    bool m_add;
};


class SetGradient : public QUndoCommand
{
public:
    typedef Document::Mesh::Curve Curve;
    typedef Document::Mesh::Value Value;
    typedef Document::Mesh::ValueFunction ValueFunction;

public:
    SetGradient(Document* doc, Curve curve, unsigned which, const ValueFunction& grad);

    virtual void undo();
    virtual void redo();

private:
    void setGradient(const ValueFunction& grad);

private:
    Document* m_document;
    Curve m_curve;
    unsigned m_which;
    ValueFunction m_prevGrad;
    ValueFunction m_newGrad;
};


class SetPointConstraintValue : public QUndoCommand
{
public:
    typedef Document::Mesh::PointConstraint PointConstraint;
    typedef Document::Mesh::Value Value;

public:
    SetPointConstraintValue(Document* doc, PointConstraint pc,
                            const Value& value);

    virtual void undo();
    virtual void redo();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    void setValue(const Value& value);

private:
    Document* m_document;
    PointConstraint m_pc;
    Value m_prevValue;
    Value m_newValue;
};


class SetPointConstraintGradient : public QUndoCommand
{
public:
    typedef Document::Mesh::PointConstraint PointConstraint;
    typedef Document::Mesh::Gradient NodeGradient;

public:
    SetPointConstraintGradient(Document* doc, PointConstraint pc,
                               const NodeGradient& gradient);

    virtual void undo();
    virtual void redo();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    void setGradient(const NodeGradient& gradient);

private:
    Document* m_document;
    PointConstraint m_pc;
    NodeGradient m_prevGradient;
    NodeGradient m_newGradient;
};


class SetCurveFlags : public QUndoCommand
{
public:
    typedef Document::Mesh Mesh;
    typedef Mesh::Curve Curve;

public:
    SetCurveFlags(Document* doc, Curve curve,
                      unsigned flags);

    virtual void undo();
    virtual void redo();

private:
    void setFlags(unsigned flags);

private:
    Document* m_document;
    Curve m_curve;
    unsigned m_prevFlags;
    unsigned m_newFlags;
};


#endif
