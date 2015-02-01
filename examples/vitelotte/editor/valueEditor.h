#ifndef VALUE_EDITOR_H
#define VALUE_EDITOR_H


#include <vector>

#include <QWidget>

#include "document.h"


class ValueEditor : public QWidget
{
    Q_OBJECT

public:
    typedef Document::Mesh Mesh;


public:
    ValueEditor(QWidget* parent = 0);
    virtual ~ValueEditor();

    virtual void resizeEvent(QResizeEvent* event);
    virtual void paintEvent(QPaintEvent* event);

    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);

    virtual QSize sizeHint() const;


public slots:
    void setDocument(Document* document);

    void updateSelection();

    void setShowMesh(int type);


private:
    struct Selection
    {
        Mesh::Halfedge h;
        Mesh::HalfedgeAttribute hn;
        Selection(Mesh::Halfedge h = Mesh::Halfedge(),
                  Mesh::HalfedgeAttribute hn = Mesh::FROM_VERTEX_VALUE);
        bool operator==(const Selection& other) const;
        bool operator!=(const Selection& other) const;
    };

    struct DisplayNode
    {
        Eigen::Vector2f pos;
        Selection sel;
    };
    typedef std::vector<DisplayNode> NodeList;

    struct DisplayEdge
    {
        int ni;
        Eigen::Vector2f dir;
        float offset;
    };
    typedef std::vector<DisplayEdge> EdgeList;

    enum NodeType
    {
        UnsetNode,
        UnknownNode,
        ConstraintNode
    };

    enum NodeSide
    {
        LeftNode = 0x01,
        RightNode = 0x02,
        CentralNode = LeftNode | RightNode
    };

private:
    QPointF vectorToPoint(const Eigen::Vector2f& v) const;
    Eigen::Vector2f pointToVector(const QPointF& p) const;
    QColor valueToColor(const Mesh::NodeValue& v) const;
    Mesh::NodeValue colorToValue(const QColor& c) const;

    NodeType nodeType(Mesh::Node n) const;
    NodeSide nodeSide(Mesh::Halfedge h, Mesh::HalfedgeAttribute hn) const;
    Mesh::NodeValue nodeValue(Mesh::Node n) const;
    Eigen::Vector2f nodePos(const Eigen::Vector2f &dir, float offset) const;
    Eigen::Vector2f gradientNodePos(NodeSide n) const;
    Eigen::Vector2f gradientHandleOffset(Mesh::Node n, int index) const;
    Eigen::Vector2f edgeValueCenter() const;
    Eigen::Vector2f edgeGradientCenter() const;

    void select(const Eigen::Vector2f& pos);
    Selection selectNode(const Eigen::Vector2f& pos) const;
    int selectGradientHandle(const Eigen::Vector2f& pos) const;

    void drawVertex(QPainter& p);
    void drawEdge(QPainter& p);
    void drawVertexValueNode(QPainter& p, const DisplayEdge& de);
    void drawNode(QPainter& p, const Eigen::Vector2f& pos,
                  Mesh::NodeValue color, Mesh::Node n, bool isSel,
                  const Eigen::Vector2f& textDir);
    void drawGradientNode(QPainter& p, const Eigen::Vector2f& pos,
                          Mesh::Halfedge h, bool isSel, NodeSide side);

    Mesh& mesh();
    const Mesh& mesh() const;

private:
    Document* m_document;
    Document::MeshType m_meshType;

    Mesh::Halfedge m_leftHalfedge;

    NodeList m_nodes;
    EdgeList m_edges;
    Selection m_selection;
    bool m_grabHandle;
    int m_selectedGradientHandle;

    QPen m_pen;

    float m_edgeOffset;
    float m_nodeOffset;
    float m_nodeSize;
    float m_textOffset;
};


#endif
