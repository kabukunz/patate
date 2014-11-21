#include <QPainter>
#include <QPaintEvent>
#include <QColorDialog>

#include "valueEditor.h"


ValueEditor::ValueEditor(QWidget* parent)
    : QWidget(parent), m_document(0),
//      m_edgeToScreen(Eigen::Matrix3f::Identity()), m_size(1), m_overNode(-1),
      m_edgeOffset(3), m_nodeOffset(16), m_nodeSize(6), m_textOffset(12)
{
    setMouseTracking(true);
}


ValueEditor::~ValueEditor()
{
}


void ValueEditor::resizeEvent(QResizeEvent* event)
{
    updateSelection();
}


void ValueEditor::paintEvent(QPaintEvent* event)
{
    QPainter p(this);
    p.setRenderHints(QPainter::Antialiasing);

    p.fillRect(event->rect(), Qt::white);

    MeshSelection sel = m_document->selection();
    if(sel.isVertex())
        drawVertex(p);
    else if(sel.isEdge())
        drawEdge(p);
}


void ValueEditor::mouseReleaseEvent(QMouseEvent* event)
{
    if(!m_document || !m_selection.h.isValid())
        return;

    Mesh& m = m_document->mesh();

    Mesh::Halfedge h = m_selection.h;
    Document::HalfedgeNode hn = m_selection.hn;

    Mesh::Node n = m_document->meshNode(h, hn);
    Mesh::Node on = m_document->meshNode(m.oppositeHalfedge(h),
                                         m_document->swapNode(hn));

    if(event->modifiers() & Qt::ControlModifier &&
            event->button() == Qt::LeftButton)
    {
        if(n == on)
        {
            if(n.isValid())
                m_document->splitNode(h, hn);
        }
        else
        {
            m_document->mergeNode(h, hn);
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

        m_document->setNodeValue(h, hn, v);
    }
}


void ValueEditor::mouseMoveEvent(QMouseEvent* event)
{
    Selection s = select(pointToVector(event->localPos()));
    if(s != m_selection)
    {
        m_selection = s;
        update();
    }
}


QSize ValueEditor::sizeHint() const
{
    return QSize(300, 300);
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
                this,         SLOT(updateSelection()));
        connect(m_document, SIGNAL(selectionChanged()),
                this,         SLOT(updateSelection()));
    }

    m_document = document;
}


void ValueEditor::updateSelection()
{
    Mesh& m = m_document->mesh();

    m_nodes.clear();
    m_edges.clear();

    m_selection.h = Mesh::Halfedge();
    if(m_document->selection().isVertex())
    {
        Mesh::Vertex v = m_document->selection().vertex();

        Mesh::HalfedgeAroundVertexCirculator hit = m.halfedges(v);
        Mesh::HalfedgeAroundVertexCirculator hend = hit;
        do
        {
            DisplayEdge edge;
            edge.dir = m.position(m.toVertex(*hit)) - m.position(m.fromVertex(*hit));
            edge.dir.normalize();
            edge.dir.y() = -edge.dir.y();

            Mesh::Node n0 = m_document->meshNode(*hit, Document::FromValueNode);
            Mesh::Node n1 = m_document->meshNode(m.oppositeHalfedge(*hit),
                                                 Document::ToValueNode);

            DisplayNode node;
            node.sel.h = *hit;
            node.sel.hn = Document::FromValueNode;
            if(n0 == n1)
            {
                edge.offset = 0;
                edge.ni = m_nodes.size();
                m_edges.push_back(edge);

                node.pos = nodePos(edge.dir, edge.offset * m_nodeOffset);
                m_nodes.push_back(node);
            }
            else
            {
                edge.offset = -1;
                edge.ni = m_nodes.size();
                m_edges.push_back(edge);

                node.pos = nodePos(edge.dir, edge.offset * m_nodeOffset);
                m_nodes.push_back(node);

                edge.offset = 1;
                edge.ni = m_nodes.size();
                m_edges.push_back(edge);

                node.sel.h = m.oppositeHalfedge(*hit);
                node.sel.hn = Document::ToValueNode;
                node.pos = nodePos(edge.dir, edge.offset * m_nodeOffset);
                m_nodes.push_back(node);
            }

            ++hit;
        } while(hit != hend);
    }
    if(m_document->selection().isEdge())
    {
        Mesh::Edge e = m_document->selection().edge();
        m_lowerHalfedge = m.halfedge(e, 0);

        Mesh::Vertex v0 = m.fromVertex(m_lowerHalfedge);
        Mesh::Vertex v1 = m.toVertex(m_lowerHalfedge);

        Eigen::Vector2f vEdge = m.position(v1) - m.position(v0);
        vEdge.y() = -vEdge.y();  // Qt y axis goes downward.

        if(vEdge.x() < 0)
        {
            m_lowerHalfedge = m.oppositeHalfedge(m_lowerHalfedge);
            vEdge = -vEdge;
        }

        vEdge.normalize();
        Eigen::Vector2f normal(-vEdge.y(), vEdge.x());

        Mesh::Node n0 = m.edgeValueNode(m_lowerHalfedge);
        Mesh::Node n1 = m.edgeValueNode(m.oppositeHalfedge(m_lowerHalfedge));

        Eigen::Vector2f center(width() / 2, height() / 2);

        DisplayNode node;
        node.sel.h = m_lowerHalfedge;
        node.sel.hn = Document::EdgeValueNode;
        if(n0 == n1)
        {
            node.pos = center;
            m_nodes.push_back(node);
        }
        else
        {
            node.pos = center - normal * m_nodeOffset;
            m_nodes.push_back(node);

            node.pos = center + normal * m_nodeOffset;
            node.sel.h = m.oppositeHalfedge(m_lowerHalfedge);
            m_nodes.push_back(node);
        }
    }

    update();
}


ValueEditor::Selection::Selection(Mesh::Halfedge h, Document::HalfedgeNode hn)
    : h(h), hn(hn)
{
}


bool ValueEditor::Selection::operator==(const Selection& other) const
{
    return h == other.h && hn == other.hn;
}


bool ValueEditor::Selection::operator!=(const Selection& other) const
{
    return !(*this == other);
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


//QPointF ValueEditor::edgeToScreen(const Eigen::Vector2f& p) const
//{
//    return vectorToPoint(
//                (m_edgeToScreen * (Eigen::Vector3f() << p, 1).finished()).head<2>());
//}


//ValueEditor::Mesh::Node ValueEditor::node(int nid) const
//{
//    Mesh::Halfedge h = m_lowerHalfedge;
//    Mesh& m = m_document->mesh();
//    Document::HalfedgeNode mnid = Document::HalfedgeNode(nid & PosMask);

//    if(nid & UpperEdge)
//    {
//        h = m_document->mesh().oppositeHalfedge(h);
//        mnid = m_document->swapNode(mnid);
//    }

//    return m_document->meshNode(h, mnid);
//}


//ValueEditor::Mesh::Node ValueEditor::oppositeNode(int nid) const
//{
//    return node(nid ^ UpperEdge);
//}


//Eigen::Vector2f ValueEditor::nodePos(int nid) const
//{
//    unsigned pos = nid & PosMask;
//    Eigen::Vector2f offset(0, 0);
//    if(node(pos) != node(pos | UpperEdge))
//        offset.y() = m_nodeOffset * (nid & UpperEdge? -1: 1);
//    switch(pos) {
//    case FromValueNode:
//        return Eigen::Vector2f( m_size, 0) + offset;
//    case ToValueNode:
//        return Eigen::Vector2f(-m_size, 0) + offset;
//    case EdgeValueNode:
//        return Eigen::Vector2f(      0, 0) + offset;
//    default: abort();
//    }
//    return Eigen::Vector2f();
//}


Eigen::Vector2f ValueEditor::nodePos(const Eigen::Vector2f& dir, float offset) const
{
    Eigen::Vector2f center(width() / 2, height() / 2);
    float nodeDist = center.minCoeff() * .6;

    Eigen::Vector2f n(-dir.y(), dir.x());
    return center + dir * nodeDist + n * offset;
}


ValueEditor::Selection ValueEditor::select(const Eigen::Vector2f& pos) const
{
    float nodeSize2 = m_nodeSize * m_nodeSize;

    for(NodeList::const_iterator dnit = m_nodes.begin();
        dnit != m_nodes.end(); ++dnit)
    {
        if(((*dnit).pos - pos).squaredNorm() <= nodeSize2)
        {
            return (*dnit).sel;
        }
    }
    return Selection();
//    if(m_lowerHalfedge.isValid())
//    {
//        float nodeSize2 = m_nodeSize * m_nodeSize;
//        for(int side=0; side<2; ++side)
//        {
//            for(int pos=0; pos<3; ++pos)
//            {
//                int nid = pos | (side? UpperEdge: 0);
//                Eigen::Vector2f np = nodePos(nid);
//                if((edgePos - np).squaredNorm() <= nodeSize2)
//                    return nid;
//            }
//        }
//    }
//    return -1;
}


//int ValueEditor::select(const QPointF& screenPos) const
//{
//    Eigen::Vector3f sp;
//    sp << pointToVector(screenPos), 1;

//    Eigen::Vector2f edgePos =
//            m_edgeToScreen.partialPivLu().solve(sp).head<2>();

//    return select(edgePos);
//}


void ValueEditor::drawVertex(QPainter& p)
{
    Mesh& m = m_document->mesh();

    Eigen::Vector2f center(width() / 2, height() / 2);

    for(EdgeList::iterator deit = m_edges.begin();
        deit != m_edges.end(); ++deit)
    {
        drawVertexValueNode(p, (*deit));
    }

    m_pen.setWidthF(1.5);
    p.setPen(m_pen);
    p.setBrush(QBrush(Qt::white));
    p.drawEllipse(vectorToPoint(center), m_nodeSize, m_nodeSize);
}


void ValueEditor::drawEdge(QPainter& p)
{
    Mesh& m = m_document->mesh();
    Mesh::Halfedge h = m_lowerHalfedge;
    Mesh::Halfedge oh = m.oppositeHalfedge(m_lowerHalfedge);

    Eigen::Vector2f center(width() / 2, height() / 2);
    Eigen::Vector2f v = (m.position(m.toVertex(h)) -
            m.position(m.fromVertex(h))).normalized();
    v.y() = -v.y();
    Eigen::Vector2f n(-v.y(), v.x());
    float edgeLen = std::min(width(), height()) * .4;

    m_pen.setWidth(2);
    p.setPen(m_pen);

    Mesh::Node lf = m_document->meshNode( h, Document::FromValueNode);
    Mesh::Node le = m_document->meshNode( h, Document::EdgeValueNode);
    Mesh::Node lt = m_document->meshNode( h, Document::ToValueNode);
    Mesh::Node uf = m_document->meshNode(oh, Document::ToValueNode);
    Mesh::Node ue = m_document->meshNode(oh, Document::EdgeValueNode);
    Mesh::Node ut = m_document->meshNode(oh, Document::FromValueNode);

    bool fSplit = (lf != uf);
    bool eSplit = (le != ue);
    bool tSplit = (lt != ut);

    p.drawLine(vectorToPoint(center - v * edgeLen - n * m_edgeOffset * fSplit),
               vectorToPoint(center               - n * m_edgeOffset * eSplit));
    if(fSplit || eSplit)
    {
        p.drawLine(vectorToPoint(center - v * edgeLen + n * m_edgeOffset * fSplit),
                   vectorToPoint(center               + n * m_edgeOffset * eSplit));
    }
    p.drawLine(vectorToPoint(center               - n * m_edgeOffset * eSplit),
               vectorToPoint(center + v * edgeLen - n * m_edgeOffset * tSplit));
    if(eSplit || tSplit)
    {
        p.drawLine(vectorToPoint(center               + n * m_edgeOffset * eSplit),
                   vectorToPoint(center + v * edgeLen + n * m_edgeOffset * tSplit));
    }

    if(eSplit)
    {
        drawValueNode(p, center - n * m_nodeOffset, le,
                      Selection(h, Document::EdgeValueNode) == m_selection, -n);
        drawValueNode(p, center + n * m_nodeOffset, ue,
                      Selection(oh, Document::EdgeValueNode) == m_selection, n);
    }
    else
        drawValueNode(p, center, le,
                      Selection(h, Document::EdgeValueNode) == m_selection, -n);
}


void ValueEditor::drawVertexValueNode(QPainter& p, const DisplayEdge& de)
{
    const DisplayNode& dn = m_nodes.at(de.ni);
    Eigen::Vector2f center(width() / 2, height() / 2);
    Eigen::Vector2f pos = dn.pos;
    Eigen::Vector2f epos = nodePos(de.dir, m_edgeOffset * de.offset);
    Eigen::Vector2f normal(-de.dir.y(), de.dir.x());
    Mesh::Node n = m_document->meshNode(dn.sel.h, dn.sel.hn);
    float nodeDist = center.minCoeff() * .2;
    bool isSel = (dn.sel == m_selection);
    Eigen::Vector2f td(-de.dir.y(), de.dir.x());
    if(de.offset < 0)
        td = -td;

    m_pen.setWidthF(1.5);
    if(n.isValid())
        m_pen.setStyle(Qt::SolidLine);
    else
        m_pen.setStyle(Qt::DashLine);
    p.setPen(m_pen);

    p.drawLine(vectorToPoint(center + normal * m_edgeOffset * de.offset),
               vectorToPoint(epos));
    p.drawLine(vectorToPoint(epos), vectorToPoint(epos + de.dir * nodeDist));

    drawValueNode(p, pos, n, isSel, td);
}


void ValueEditor::drawValueNode(QPainter& p, const Eigen::Vector2f& pos,
                                Mesh::Node n, bool isSel,
                                const Eigen::Vector2f& textDir)
{
    m_pen.setStyle(Qt::SolidLine);
    if(!n.isValid())
    {
        float size = isSel? m_nodeSize: m_nodeSize / 2.;
        m_pen.setWidthF(0);
        p.setPen(m_pen);
        p.setBrush(Qt::black);
        p.drawEllipse(vectorToPoint(pos), size, size);
    }
    else if(m_document->mesh().isConstraint(n))
    {
        m_pen.setWidthF(isSel? 2: 1);
        p.setPen(m_pen);

        p.setBrush(QBrush(valueToColor(m_document->mesh().nodeValue(n))));
        p.drawEllipse(vectorToPoint(pos), m_nodeSize, m_nodeSize);
    }
    else
    {
        m_pen.setWidthF(isSel? 2: 1.2);
        p.setPen(m_pen);

        p.drawLine(vectorToPoint(pos + Eigen::Vector2f(-m_nodeSize, -m_nodeSize)),
                   vectorToPoint(pos + Eigen::Vector2f( m_nodeSize,  m_nodeSize)));
        p.drawLine(vectorToPoint(pos + Eigen::Vector2f(-m_nodeSize,  m_nodeSize)),
                   vectorToPoint(pos + Eigen::Vector2f( m_nodeSize, -m_nodeSize)));
    }

    QFontMetrics fm(p.font());
    QString id = QString::number(n.idx());
    Eigen::Vector2f textHalfSize =
            Eigen::Vector2f(fm.width(id), fm.ascent()-fm.descent()) / 2.;
    Eigen::Vector2f textOffset = textDir * m_textOffset
            + textHalfSize.asDiagonal() * textDir;
    textHalfSize.y() = -textHalfSize.y();
    QPointF posText = vectorToPoint(pos + textOffset - textHalfSize);
    p.drawText(posText, id);

//    m_pen.setWidthF(0);
//    p.setPen(m_pen);
//    p.setBrush(Qt::blue);
//    p.drawEllipse(vectorToPoint(pos + textDir * m_textOffset), 2, 2);
//    p.setBrush(Qt::green);
//    p.drawEllipse(vectorToPoint(pos + textOffset), 2, 2);
}
