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

    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);

    virtual QSize sizeHint() const;


public slots:
    void setDocument(Document* document);

    void updateSelection();


private:
    struct Selection
    {
        Mesh::Halfedge h;
        Document::HalfedgeNode hn;
        Selection(Mesh::Halfedge h = Mesh::Halfedge(),
                  Document::HalfedgeNode hn = Document::FromValueNode);
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

private:
    QPointF vectorToPoint(const Eigen::Vector2f& v) const;
    Eigen::Vector2f pointToVector(const QPointF& p) const;
    QTransform matrixToTransform(const Eigen::Matrix3f& m) const;
    QColor valueToColor(const Mesh::NodeValue& v) const;
    Mesh::NodeValue colorToValue(const QColor& c) const;
//    QPointF edgeToScreen(const Eigen::Vector2f& p) const;

//    Mesh::Node node(int nid) const;
//    Mesh::Node oppositeNode(int nid) const;
//    Eigen::Vector2f nodePos(int nid) const;
    Eigen::Vector2f nodePos(const Eigen::Vector2f &dir, float offset) const;

    Selection select(const Eigen::Vector2f& pos) const;
//    int select(const QPointF& screenPos) const;

    void drawVertex(QPainter& p);
    void drawEdge(QPainter& p);
    void drawVertexValueNode(QPainter& p, const DisplayEdge& de);
    void drawValueNode(QPainter& p, const Eigen::Vector2f& pos,
                       Mesh::Node n, bool isSel, const Eigen::Vector2f& textDir);


private:
    Document* m_document;

//    Eigen::Matrix3f m_edgeToScreen;
//    float m_size;
    Mesh::Halfedge m_lowerHalfedge;
//    int m_overNode;

    NodeList m_nodes;
    EdgeList m_edges;
    Selection m_selection;

    QPen m_pen;

    float m_edgeOffset;
    float m_nodeOffset;
    float m_nodeSize;
    float m_textOffset;
};


#endif
