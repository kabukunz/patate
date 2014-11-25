#include <QPainter>
#include <QPaintEvent>
#include <QColorDialog>

#include "valueEditor.h"


ValueEditor::ValueEditor(QWidget* parent)
    : QWidget(parent), m_document(0),
      m_selection(), m_grabHandle(false), m_selectedGradientHandle(-1),
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


void ValueEditor::mousePressEvent(QMouseEvent* event)
{
    if(!m_document)
        return;

    if(m_selectedGradientHandle >= 0 && event->button() == Qt::LeftButton)
    {
        m_grabHandle = true;
        grabMouse();

        Mesh::Halfedge h = m_leftHalfedge;
        if(m_selectedGradientHandle > 3)
            h = m_document->mesh().oppositeHalfedge(h);
        Mesh::Node n = m_document->meshNode(h, Document::EdgeGradientNode);
        m_document->setNodeValue(h, Document::EdgeGradientNode,
                                 m_document->mesh().nodeValue(n), false);
    }
}


void ValueEditor::mouseReleaseEvent(QMouseEvent* event)
{
    if(!m_document)
        return;

    Mesh& m = m_document->mesh();

    if(m_grabHandle)
    {
        releaseMouse();
        m_grabHandle = false;

        return;
    }

    if(m_selection.h.isValid())
    {
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
                if(hn != Document::EdgeGradientNode)
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
                else
                    v = Mesh::NodeValue::Zero();
            }

            m_document->setNodeValue(h, hn, v);
        }
    }
}


void ValueEditor::mouseMoveEvent(QMouseEvent* event)
{
    if(!m_document)
        return;

    Eigen::Vector2f cursor = pointToVector(event->localPos());
    if(!m_grabHandle)
    {
        select(cursor);
    }
    else
    {
        assert(m_selectedGradientHandle >= 0 && m_selectedGradientHandle < 8);
        Mesh::Halfedge h = m_leftHalfedge;
        int index = m_selectedGradientHandle;

        if(index > 3)
        {
            h = m_document->mesh().oppositeHalfedge(h);
            index -= 4;
        }

        Mesh::Node n = m_document->meshNode(h, Document::EdgeGradientNode);
        Mesh::NodeValue value = m_document->mesh().nodeValue(n);
        Eigen::Vector2f v = cursor -
                gradientNodePos(nodeSide(h, Document::EdgeGradientNode));

        value(index) = -v.y() / std::max(std::abs(v.x()), 1.f);
        if(m_document->mesh().halfedgeOrientation(h))
            value(index) = -value(index);

        m_document->setNodeValue(h, Document::EdgeGradientNode, value, true);
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
        Mesh::Halfedge lh = m.halfedge(e, 0);

        Mesh::Vertex v0 = m.fromVertex(lh);
        Mesh::Vertex v1 = m.toVertex(lh);

        Eigen::Vector2f vEdge = m.position(v1) - m.position(v0);
        vEdge.y() = -vEdge.y();  // Qt y axis goes downward.

        if(vEdge.y() >= 0)
        {
            lh = m.oppositeHalfedge(lh);
            vEdge = -vEdge;
        }
        Mesh::Halfedge rh = m.oppositeHalfedge(lh);
        m_leftHalfedge = lh;

        vEdge.normalize();
        Eigen::Vector2f normal(-vEdge.y(), vEdge.x());

        Mesh::Node ln = m.edgeValueNode(lh);
        Mesh::Node rn = m.edgeValueNode(rh);

        Eigen::Vector2f center = edgeValueCenter();

        DisplayNode node;
        node.sel.h = lh;
        node.sel.hn = Document::EdgeValueNode;
        if(ln == rn)
        {
            node.pos = center;
            m_nodes.push_back(node);
        }
        else
        {
            node.pos = center - normal * m_nodeOffset;
            m_nodes.push_back(node);

            node.pos = center + normal * m_nodeOffset;
            node.sel.h = rh;
            m_nodes.push_back(node);
        }

        // Gradient node
        Mesh::Node lgn = m.edgeGradientNode(lh);
        Mesh::Node rgn = m.edgeGradientNode(rh);

        node.sel.h = lh;
        node.sel.hn = Document::EdgeGradientNode;
        if(lgn == rgn)
        {
            node.pos = gradientNodePos(CentralNode);
            m_nodes.push_back(node);
        }
        else
        {
            node.pos = gradientNodePos(LeftNode);
            m_nodes.push_back(node);

            node.pos = gradientNodePos(RightNode);
            node.sel.h = rh;
            m_nodes.push_back(node);
        }
    }

    if(!m_grabHandle)
    {
        Eigen::Vector2f pos = pointToVector(QPointF(mapFromGlobal(QCursor::pos())));
        select(pos);
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


ValueEditor::NodeType ValueEditor::nodeType(Mesh::Node n) const
{
    if(n.idx() < 0 )
        return UnsetNode;
    else if(m_document->mesh().isConstraint(n))
        return ConstraintNode;
    return UnknownNode;
}


ValueEditor::NodeSide ValueEditor::nodeSide(
        Mesh::Halfedge h, Document::HalfedgeNode hn) const
{
    Mesh::Halfedge oh = m_document->mesh().oppositeHalfedge(h);
    if(m_document->meshNode(h, hn) == m_document->meshNode(oh, hn))
        return CentralNode;
    else if(h == m_leftHalfedge)
        return LeftNode;
    return RightNode;
}


ValueEditor::Mesh::NodeValue ValueEditor::nodeValue(Mesh::Node n) const
{
    if(n.isValid() && m_document->mesh().isConstraint(n))
        return m_document->mesh().nodeValue(n);
    return Mesh::NodeValue(0, 0, 0, 1);
}


Eigen::Vector2f ValueEditor::nodePos(const Eigen::Vector2f& dir, float offset) const
{
    Eigen::Vector2f center(width() / 2, height() / 2);
    float nodeDist = center.minCoeff() * .6;

    Eigen::Vector2f n(-dir.y(), dir.x());
    return center + dir * nodeDist + n * offset;
}


Eigen::Vector2f ValueEditor::gradientNodePos(NodeSide side) const
{
    Eigen::Vector2f pos = edgeGradientCenter();
    if(side == LeftNode)
        pos.x() -= m_nodeOffset;
    else if(side == RightNode)
        pos.x() += m_nodeOffset;
    return pos;
}


Eigen::Vector2f ValueEditor::gradientHandleOffset(
        Mesh::Node n, int index) const
{
    Eigen::Vector2f v(1, nodeValue(n)(index));
    v.normalize();

    if(m_document->mesh().halfedgeOrientation(m_leftHalfedge))
        v.y() = -v.y();

    float baseLen = edgeValueCenter().minCoeff() * .7;
    float len = baseLen - index * (baseLen / 9);
    return v*len;
}


Eigen::Vector2f ValueEditor::edgeValueCenter() const
{
    int w = width();
    int h = height();
    if(w > h) return Eigen::Vector2f(w/4, h/2);
    else      return Eigen::Vector2f(w/2, h/4);
}


Eigen::Vector2f ValueEditor::edgeGradientCenter() const
{
    int w = width();
    int h = height();
    if(w > h) return Eigen::Vector2f(w*3/4, h/2);
    else      return Eigen::Vector2f(w/2, h*3/4);
}


void ValueEditor::select(const Eigen::Vector2f& pos)
{
    Selection s = selectNode(pos);
    if(s != m_selection)
    {
        m_selection = s;
        m_selectedGradientHandle = -1;
        update();
    }

    if(!m_selection.h.isValid());
    {
        int h = selectGradientHandle(pos);
        if(h != m_selectedGradientHandle)
        {
            m_selectedGradientHandle = h;
            update();
        }
    }
}


ValueEditor::Selection ValueEditor::selectNode(const Eigen::Vector2f& pos) const
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
}


int ValueEditor::selectGradientHandle(const Eigen::Vector2f& pos) const
{
    float minSqrDist = 8*8;
    int candidate = -1;
    if(m_leftHalfedge.isValid())
    {
        for(int side = 0; side < 2; ++side)
        {
            Mesh::Halfedge h = m_leftHalfedge;
            if(side) h = m_document->mesh().oppositeHalfedge(h);
            Eigen::Vector2f base = gradientNodePos(
                        nodeSide(h, Document::EdgeGradientNode));
            Mesh::Node n = m_document->meshNode(h, Document::EdgeGradientNode);
            float factor = side? 1: -1;

            for(int i = 0; i < 4; ++i)
            {
                Eigen::Vector2f off = gradientHandleOffset(n, i) * factor;
                float dist = (base + off - pos).squaredNorm();
                if(dist <= minSqrDist)
                {
                    minSqrDist = dist;
                    candidate = i + 4*side;
                }
            }
        }
    }
    return candidate;
}


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
    Mesh::Halfedge lh = m_leftHalfedge;
    Mesh::Halfedge rh = m.oppositeHalfedge(m_leftHalfedge);

    Eigen::Vector2f center = edgeValueCenter();
    Eigen::Vector2f v = (m.position(m.toVertex(lh)) -
            m.position(m.fromVertex(lh))).normalized();
    v.y() = -v.y();
    Eigen::Vector2f n(-v.y(), v.x());
    float edgeLen = center.minCoeff() * .8;

    m_pen.setWidth(2);
    p.setPen(m_pen);

    Mesh::Node lf = m_document->meshNode(lh, Document::FromValueNode);
    Mesh::Node le = m_document->meshNode(lh, Document::EdgeValueNode);
    Mesh::Node lt = m_document->meshNode(lh, Document::ToValueNode);
    Mesh::Node rf = m_document->meshNode(rh, Document::ToValueNode);
    Mesh::Node re = m_document->meshNode(rh, Document::EdgeValueNode);
    Mesh::Node rt = m_document->meshNode(rh, Document::FromValueNode);

    bool fSplit = (lf != rf);
    bool eSplit = (le != re);
    bool tSplit = (lt != rt);

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
        drawNode(p, center - n * m_nodeOffset, nodeValue(le), le,
                 Selection(lh, Document::EdgeValueNode) == m_selection, -n);
        drawNode(p, center + n * m_nodeOffset, nodeValue(re), re,
                 Selection(rh, Document::EdgeValueNode) == m_selection, n);
    }
    else
        drawNode(p, center, nodeValue(le), le,
                 Selection(lh, Document::EdgeValueNode) == m_selection, -n);

    Mesh::Node lg = m_document->meshNode(lh, Document::EdgeGradientNode);
    Mesh::Node rg = m_document->meshNode(rh, Document::EdgeGradientNode);

    if(lg == rg)
    {
        drawGradientNode(p, gradientNodePos(CentralNode), lh,
                         Selection(lh, Document::EdgeGradientNode) == m_selection,
                         CentralNode);
    }
    else
    {
        drawGradientNode(p, gradientNodePos(LeftNode), lh,
                         Selection(lh, Document::EdgeGradientNode) == m_selection,
                         LeftNode);
        drawGradientNode(p, gradientNodePos(RightNode), rh,
                         Selection(rh, Document::EdgeGradientNode) == m_selection,
                         RightNode);
    }
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

    drawNode(p, pos, nodeValue(n), n, isSel, td);
}


void ValueEditor::drawNode(QPainter& p, const Eigen::Vector2f& pos,
                           Mesh::NodeValue color, Mesh::Node n, bool isSel,
                           const Eigen::Vector2f& textDir)
{
    m_pen.setStyle(Qt::SolidLine);
    switch(nodeType(n))
    {
    case UnsetNode:
    {
        float size = isSel? m_nodeSize: m_nodeSize / 2.;
        m_pen.setWidthF(0);
        p.setPen(m_pen);
        p.setBrush(Qt::black);
        p.drawEllipse(vectorToPoint(pos), size, size);
        break;
    }
    case UnknownNode:
    {
        m_pen.setWidthF(isSel? 2: 1.2);
        p.setPen(m_pen);

        p.drawLine(vectorToPoint(pos + Eigen::Vector2f(-m_nodeSize, -m_nodeSize)),
                   vectorToPoint(pos + Eigen::Vector2f( m_nodeSize,  m_nodeSize)));
        p.drawLine(vectorToPoint(pos + Eigen::Vector2f(-m_nodeSize,  m_nodeSize)),
                   vectorToPoint(pos + Eigen::Vector2f( m_nodeSize, -m_nodeSize)));
        break;
    }
    case ConstraintNode:
    {
        m_pen.setWidthF(isSel? 2: 1);
        p.setPen(m_pen);

        p.setBrush(QBrush(valueToColor(color)));
        p.drawEllipse(vectorToPoint(pos), m_nodeSize, m_nodeSize);
        break;
    }
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


void ValueEditor::drawGradientNode(QPainter& p, const Eigen::Vector2f& pos,
                                   Mesh::Halfedge h, bool isSel,
                                   NodeSide side)
{
    Mesh::Node n = m_document->meshNode(h, Document::EdgeGradientNode);
    if(n.isValid() && m_document->mesh().isConstraint(n))
    {
        Eigen::Vector4f value = nodeValue(n);
        for(int i = 0; i < 4; ++i)
        {
            QColor color = QColor::fromRgbF(i == 0, i == 1, i == 2);
            m_pen.setWidthF(1.2);
            m_pen.setColor(color);
            p.setPen(m_pen);

            p.setBrush(QBrush(color));

            Eigen::Vector2f v = gradientHandleOffset(n, i);

            if(side & LeftNode)
            {
                float r = (m_selectedGradientHandle == i)? 3: 2;
                p.drawLine(vectorToPoint(pos), vectorToPoint(pos - v));
                p.drawEllipse(vectorToPoint(pos - v), r, r);
            }
            if(side & RightNode)
            {
                float r = (m_selectedGradientHandle == i+4)? 3: 2;
                p.drawLine(vectorToPoint(pos), vectorToPoint(pos + v));
                p.drawEllipse(vectorToPoint(pos + v), r, r);
            }
        }
    }

    m_pen.setColor(Qt::black);
    drawNode(p, pos, Mesh::NodeValue(1, 1, 1, 1), n, isSel,
             Eigen::Vector2f(0, (side & RightNode)? -1: 1));
}
