#include <QPainter>
#include <QPaintEvent>

#include "valueEditor.h"


ValueEditor::ValueEditor(QWidget* parent)
    : QWidget(parent), m_document(0),
      m_edgeToScreen(Eigen::Matrix3f::Identity()), m_size(1),
      m_edgeOffset(3), m_nodeOffset(20), m_nodeSize(6), m_textOffset(20)
{
}


ValueEditor::~ValueEditor()
{
}


void ValueEditor::resizeEvent(QResizeEvent* event)
{
    m_edgeToScreen(0, 2) = event->size().width() / 2;
    m_edgeToScreen(1, 2) = event->size().height() / 2;
    m_size = std::min(event->size().width(), event->size().height()) / 2.2;
}


void ValueEditor::paintEvent(QPaintEvent* event)
{
    QPainter p(this);
    p.setRenderHints(QPainter::Antialiasing);

    p.fillRect(event->rect(), Qt::white);

    if(m_lowerHalfedge.isValid())
    {
        Mesh& m = m_document->mesh();

        Mesh::Halfedge h0 = m_lowerHalfedge;
        Mesh::Halfedge h1 = m.oppositeHalfedge(m_lowerHalfedge);

        bool sSplit = m.vertexValueNode(h0) != m.vertexFromValueNode(h1);
        bool mSplit = m.edgeValueNode(h0) != m.edgeValueNode(h1);
        bool eSplit = m.vertexFromValueNode(h0) != m.vertexValueNode(h1);

        p.setWorldTransform(matrixToTransform(m_edgeToScreen));

        m_pen.setWidth(2);
        p.setPen(m_pen);

        p.drawLine(QPointF(-m_size, sSplit?  m_edgeOffset: 0),
                   QPointF(      0, mSplit?  m_edgeOffset: 0));
        if(sSplit || mSplit)
        {
            p.drawLine(QPointF(-m_size, sSplit? -m_edgeOffset: 0),
                       QPointF(      0, mSplit? -m_edgeOffset: 0));
        }
        p.drawLine(QPointF(      0, mSplit?  m_edgeOffset: 0),
                   QPointF( m_size, eSplit?  m_edgeOffset: 0));
        if(mSplit || eSplit)
        {
            p.drawLine(QPointF(      0, mSplit? -m_edgeOffset: 0),
                       QPointF( m_size, eSplit? -m_edgeOffset: 0));
        }

        drawValueNode(
            p, m.vertexValueNode(h0), QPointF(-m_size, 0), sSplit);
        if(sSplit)
            drawValueNode(p, m.vertexFromValueNode(h1), QPointF(-m_size, 0), -1);

        drawValueNode(p, m.edgeValueNode(h0), QPointF(0, 0), mSplit);
        if(mSplit)
            drawValueNode(p, m.edgeValueNode(h1), QPointF(     0, 0), -1);

        drawValueNode(p, m.vertexFromValueNode(h0), QPointF( m_size, 0), eSplit);
        if(eSplit)
            drawValueNode(p, m.vertexValueNode(h1), QPointF( m_size, 0), -1);
    }
}


void ValueEditor::setDocument(Document* document)
{
    if(m_document)
    {
        disconnect(m_document);
        m_document->disconnect(this);
    }

    m_document = document;
    update();

    if(m_document)
    {
        connect(m_document, SIGNAL(selectedEdgeChanged()),
                this,         SLOT(updateSelection()));
    }

    m_document = document;
}


void ValueEditor::updateSelection()
{
    Mesh& m = m_document->mesh();

    Mesh::Edge e = m_document->selectedEdge();
    if(!m.isValid(e))
    {
        m_lowerHalfedge = Mesh::Halfedge();
        update();
        return;
    }

    Mesh::Vertex v0 = m.vertex(e, 0);
    Mesh::Vertex v1 = m.vertex(e, 1);

    Eigen::Vector2f vEdge = m.position(v1) - m.position(v0);
    vEdge.y() = -vEdge.y();  // Qt y axis goes downward.

    if(vEdge.x() < 0)
    {
        m_lowerHalfedge = m.halfedge(e, 1);
        vEdge = -vEdge;
    }
    else
        m_lowerHalfedge = m.halfedge(e, 0);

    vEdge.normalize();
    Eigen::Vector2f normal(-vEdge.y(), vEdge.x());

    m_edgeToScreen.topLeftCorner<2, 2>() <<
            vEdge.normalized(), normal;

    update();
}


QPointF ValueEditor::vectorToPoint(const Eigen::Vector2f& v) const
{
    return QPointF(v.x(), v.y());
}


QTransform ValueEditor::matrixToTransform(const Eigen::Matrix3f& m) const
{
    return QTransform(m(0, 0), m(1, 0), m(2, 0),
                      m(0, 1), m(1, 1), m(2, 1),
                      m(0, 2), m(1, 2), m(2, 2));
}


QColor ValueEditor::valueToColor(const Mesh::NodeValue& v) const
{
    return QColor(
                std::min(std::max(int(v(0) * 256), 0), 255),
                std::min(std::max(int(v(1) * 256), 0), 255),
                std::min(std::max(int(v(2) * 256), 0), 255),
                std::min(std::max(int(v(3) * 256), 0), 255));
}


ValueEditor::Mesh::Node ValueEditor::node(unsigned node) const
{
    Mesh::Halfedge h = m_lowerHalfedge;
    Mesh& m = m_document->mesh();
    if(node & UpperEdge)
        h = m_document->mesh().oppositeHalfedge(h);

    switch(node) {
    case (FromNode | LowerEdge):
    case (ToNode   | UpperEdge):
        return m.vertexValueNode(h);
    case (MidNode  | LowerEdge):
    case (MidNode  | UpperEdge):
        return m.edgeValueNode(h);
    case (ToNode   | LowerEdge):
    case (FromNode | UpperEdge):
        return m.vertexFromValueNode(h);
    default: abort();
    }
    return Mesh::Node();
}


Eigen::Vector2f ValueEditor::nodePos(unsigned node) const
{
    Eigen::Vector2f offset(0, m_nodeOffset * (node & LowerEdge? 1: -1));
    switch(node & PosMask) {
    case FromNode:
        return Eigen::Vector2f(-m_size, 0) + offset;
    case MidNode:
        return Eigen::Vector2f(      0, 0) + offset;
    case ToNode:
        return Eigen::Vector2f( m_size, 0) + offset;
    default: abort();
    }
    return Eigen::Vector2f();
}


void ValueEditor::drawValueNode(QPainter& p, Mesh::Node n, const QPointF& pos,
                                float offset)
{
    offset *= m_nodeOffset;

    QPointF pos2 = pos + QPointF(0, offset);
    if(n.isValid() && m_document->mesh().isConstraint(n))
    {
        m_pen.setWidth(1);
        p.setPen(m_pen);

        p.setBrush(QBrush(valueToColor(m_document->mesh().nodeValue(n))));
        p.drawEllipse(pos2, m_nodeSize, m_nodeSize);
    }
    else
    {
        m_pen.setWidth(2);
        p.setPen(m_pen);

        p.drawLine(pos2 + QPointF(-m_nodeSize, -m_nodeSize),
                   pos2 + QPointF( m_nodeSize,  m_nodeSize));
        p.drawLine(pos2 + QPointF(-m_nodeSize,  m_nodeSize),
                   pos2 + QPointF( m_nodeSize, -m_nodeSize));
    }

    QFontMetrics fm(p.font());
    QString id = QString::number(n.idx());
    float textOffset = m_textOffset * (offset>0? 1: -1);
    QPointF pos3 = pos2 + QPointF(-fm.width(id) / 2,
        textOffset + (offset>0? fm.ascent(): 0));
    p.drawText(pos3, id);
}
