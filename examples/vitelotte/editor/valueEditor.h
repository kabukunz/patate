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


public slots:
    void setDocument(Document* document);

    void updateSelection();


private:
    enum InnerSelection
    {
        FromNode  = 0x00,
        MidNode   = 0x01,
        ToNode    = 0x02,
        PosMask   = 0x03,

        LowerEdge = 0x00,
        UpperEdge = 0x04
    };

private:
    QPointF vectorToPoint(const Eigen::Vector2f& v) const;
    QTransform matrixToTransform(const Eigen::Matrix3f& m) const;
    QColor valueToColor(const Mesh::NodeValue& v) const;

    Mesh::Node node(unsigned node) const;
    Eigen::Vector2f nodePos(unsigned node) const;

    void drawValueNode(QPainter& p, Mesh::Node n, const QPointF& pos,
                       float offset);


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
