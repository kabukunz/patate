#include <QPainter>
#include <QPaintEvent>
#include <QColorDialog>

#include "valueEditor.h"


ValueEditor::ValueEditor(QWidget* parent)
    : QWidget(parent), m_document(0),
      m_edgeToScreen(Eigen::Matrix3f::Identity()), m_size(1), m_overNode(-1),
      m_edgeOffset(3), m_nodeOffset(20), m_nodeSize(6), m_textOffset(20)
{
    setMouseTracking(true);
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
        //p.setWorldTransform(matrixToTransform(m_edgeToScreen));

        m_pen.setWidth(2);
        p.setPen(m_pen);

        bool fSplit = node(FromValueNode) != node(FromValueNode | UpperEdge);
        bool tSplit = node(  ToValueNode) != node(  ToValueNode | UpperEdge);
        bool eSplit = node(EdgeValueNode) != node(EdgeValueNode | UpperEdge);

        p.drawLine(edgeToScreen(Eigen::Vector2f( m_size, fSplit *  m_edgeOffset)),
                   edgeToScreen(Eigen::Vector2f(      0, eSplit *  m_edgeOffset)));
        if(fSplit || eSplit)
        {
            p.drawLine(edgeToScreen(Eigen::Vector2f( m_size, fSplit * -m_edgeOffset)),
                       edgeToScreen(Eigen::Vector2f(      0, eSplit * -m_edgeOffset)));
        }
        p.drawLine(edgeToScreen(Eigen::Vector2f(      0, eSplit *  m_edgeOffset)),
                   edgeToScreen(Eigen::Vector2f(-m_size, tSplit *  m_edgeOffset)));
        if(eSplit || tSplit)
        {
            p.drawLine(edgeToScreen(Eigen::Vector2f(      0, eSplit * -m_edgeOffset)),
                       edgeToScreen(Eigen::Vector2f(-m_size, tSplit * -m_edgeOffset)));
        }

        for(int i=0; i<3; ++i)
        {
            Mesh::Node nl = node(i);
            Mesh::Node nu = node(i | UpperEdge);

            drawValueNode(p, i);
            if(nl != nu)
                drawValueNode(p, i | UpperEdge);
        }
    }
}


void ValueEditor::mouseReleaseEvent(QMouseEvent* event)
{
    if(!m_document || m_overNode < 0)
        return;

    Mesh& m = m_document->mesh();

    Mesh::Node n = node(m_overNode);
    Mesh::Node on = oppositeNode(m_overNode);

    Mesh::Halfedge h = m_lowerHalfedge;
    Document::HalfedgeNode nid = Document::HalfedgeNode(m_overNode & PosMask);
    if(m_overNode & UpperEdge)
    {
        h = m.oppositeHalfedge(h);
        nid = m_document->swapNode(nid);
    }

    if(event->modifiers() & Qt::ControlModifier &&
            event->button() == Qt::LeftButton)
    {
        if(n == on)
        {
            if(n.isValid())
                m_document->splitNode(h, nid);
        }
        else
        {
            m_document->mergeNode(h, nid);
        }
    }
    else if(event->button() == Qt::LeftButton || event->button() == Qt::RightButton)
    {
        Mesh::NodeValue v = Mesh::UnconstrainedNode;
        if(event->button() == Qt::LeftButton)
        {
            QColor color = Qt::white;
            if(n.isValid())
                color = valueToColor(m.nodeValue(n));
            color = QColorDialog::getColor(color, this,
                    "Pick a color", QColorDialog::ShowAlphaChannel);
            if(!color.isValid())
                return;
            v = colorToValue(color);
        }

        m_document->setNodeValue(h, nid, v);
    }
}


void ValueEditor::mouseMoveEvent(QMouseEvent* event)
{
    int s = select(event->localPos());
    if(s != m_overNode)
    {
        m_overNode = s;
        update();
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
        connect(m_document, SIGNAL(meshUpdated()),
                this,         SLOT(update()));
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


Eigen::Vector2f ValueEditor::pointToVector(const QPointF& p) const
{
    return Eigen::Vector2f(p.x(), p.y());
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
                std::min(std::max(int(v(0) * 255), 0), 255),
                std::min(std::max(int(v(1) * 255), 0), 255),
                std::min(std::max(int(v(2) * 255), 0), 255),
                std::min(std::max(int(v(3) * 255), 0), 255));
}


ValueEditor::Mesh::NodeValue ValueEditor::colorToValue(const QColor& c) const
{
    return Mesh::NodeValue(
                c.redF(), c.greenF(), c.blueF(), c.alphaF());
}


QPointF ValueEditor::edgeToScreen(const Eigen::Vector2f& p) const
{
    return vectorToPoint(
                (m_edgeToScreen * (Eigen::Vector3f() << p, 1).finished()).head<2>());
}


ValueEditor::Mesh::Node ValueEditor::node(int nid) const
{
    Mesh::Halfedge h = m_lowerHalfedge;
    Mesh& m = m_document->mesh();
    Document::HalfedgeNode mnid = Document::HalfedgeNode(nid & PosMask);

    if(nid & UpperEdge)
    {
        h = m_document->mesh().oppositeHalfedge(h);
        mnid = m_document->swapNode(mnid);
    }

    return m_document->meshNode(h, mnid);
}


ValueEditor::Mesh::Node ValueEditor::oppositeNode(int nid) const
{
    return node(nid ^ UpperEdge);
}


Eigen::Vector2f ValueEditor::nodePos(int nid) const
{
    unsigned pos = nid & PosMask;
    Eigen::Vector2f offset(0, 0);
    if(node(pos) != node(pos | UpperEdge))
        offset.y() = m_nodeOffset * (nid & UpperEdge? -1: 1);
    switch(pos) {
    case FromValueNode:
        return Eigen::Vector2f( m_size, 0) + offset;
    case ToValueNode:
        return Eigen::Vector2f(-m_size, 0) + offset;
    case EdgeValueNode:
        return Eigen::Vector2f(      0, 0) + offset;
    default: abort();
    }
    return Eigen::Vector2f();
}


int ValueEditor::select(const Eigen::Vector2f& edgePos) const
{
    if(m_lowerHalfedge.isValid())
    {
        float nodeSize2 = m_nodeSize * m_nodeSize;
        for(int side=0; side<2; ++side)
        {
            for(int pos=0; pos<3; ++pos)
            {
                int nid = pos | (side? UpperEdge: 0);
                Eigen::Vector2f np = nodePos(nid);
                if((edgePos - np).squaredNorm() <= nodeSize2)
                    return nid;
            }
        }
    }
    return -1;
}


int ValueEditor::select(const QPointF& screenPos) const
{
    Eigen::Vector3f sp;
    sp << pointToVector(screenPos), 1;

    Eigen::Vector2f edgePos =
            m_edgeToScreen.partialPivLu().solve(sp).head<2>();

    return select(edgePos);
}


void ValueEditor::drawValueNode(QPainter& p, int nid)
{
    Mesh::Node n = node(nid);
    Eigen::Vector2f edgePos = nodePos(nid);
    if(n.isValid() && m_document->mesh().isConstraint(n))
    {
        m_pen.setWidth(nid == m_overNode? 2: 1);
        p.setPen(m_pen);

        p.setBrush(QBrush(valueToColor(m_document->mesh().nodeValue(n))));
        p.drawEllipse(edgeToScreen(edgePos), m_nodeSize, m_nodeSize);
    }
    else
    {
        m_pen.setWidthF(nid == m_overNode? 2: 1.2);
        p.setPen(m_pen);

        p.drawLine(edgeToScreen(edgePos + Eigen::Vector2f(-m_nodeSize, -m_nodeSize)),
                   edgeToScreen(edgePos + Eigen::Vector2f( m_nodeSize,  m_nodeSize)));
        p.drawLine(edgeToScreen(edgePos + Eigen::Vector2f(-m_nodeSize,  m_nodeSize)),
                   edgeToScreen(edgePos + Eigen::Vector2f( m_nodeSize, -m_nodeSize)));
    }

    QFontMetrics fm(p.font());
    QString id = QString::number(n.idx());
    float textOffset = m_textOffset * (edgePos.y()>0? 1: -1);
//    QPointF posText = pos + QPointF(-fm.width(id) / 2,
//        textOffset + (pos.y()>0? fm.ascent(): 0));
    QPointF posText =
            edgeToScreen(edgePos +
                         Eigen::Vector2f(-fm.width(id) / 2., textOffset)) +
            QPointF(0, fm.ascent()/2);
    p.drawText(posText, id);
}
