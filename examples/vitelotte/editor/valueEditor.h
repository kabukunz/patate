#ifndef VALUE_EDITOR_H
#define VALUE_EDITOR_H


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


public slots:
    void setDocument(Document* document);

    void updateSelection();


private:
    enum InnerSelection
    {
        FromValueNode    = Document::FromValueNode,
        ToValueNode      = Document::ToValueNode,
        EdgeValueNode    = Document::EdgeValueNode,
        EdgeGradientNode = Document::EdgeGradientNode,

        PosMask = 0x0f,

        UpperEdge = 0x10
    };

private:
    QPointF vectorToPoint(const Eigen::Vector2f& v) const;
    Eigen::Vector2f pointToVector(const QPointF& p) const;
    QTransform matrixToTransform(const Eigen::Matrix3f& m) const;
    QColor valueToColor(const Mesh::NodeValue& v) const;
    Mesh::NodeValue colorToValue(const QColor& c) const;

    Mesh::Node node(int nid) const;
    Mesh::Node oppositeNode(int nid) const;
    Eigen::Vector2f nodePos(int nid) const;

    int select(const Eigen::Vector2f& edgePos) const;
    int select(const QPointF& screenPos) const;

    void drawValueNode(QPainter& p, int nid);


private:
    Document* m_document;

    Eigen::Matrix3f m_edgeToScreen;
    float m_size;
    Mesh::Halfedge m_lowerHalfedge;
    int m_overNode;

    QPen m_pen;

    float m_edgeOffset;
    float m_nodeOffset;
    float m_nodeSize;
    float m_textOffset;
};


#endif
