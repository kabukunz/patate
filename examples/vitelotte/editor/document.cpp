#include <QFileDialog>

#include "Patate/vitelotte_io.h"

#include "document.h"


MeshSelection::MeshSelection()
    : m_type(SelectionNone),
      m_index(-1)
{}


MeshSelection::MeshSelection(Mesh::Vertex v)
    : m_type(v.isValid()? SelectionVertex: SelectionNone),
      m_index(v.idx())
{}


MeshSelection::MeshSelection(Mesh::Edge e)
    : m_type(e.isValid()? SelectionEdge: SelectionNone),
      m_index(e.idx())
{}


bool MeshSelection::operator==(const MeshSelection& other) const
{
    return m_type == other.type() && m_index == other.m_index;
}


bool MeshSelection::operator!=(const MeshSelection& other) const
{
    return !(*this == other);
}


MeshSelection::SelectionType MeshSelection::type() const
{
    return m_type;
}


bool MeshSelection::isNone() const
{
    return type() == SelectionNone;
}


bool MeshSelection::isVertex() const
{
    return type() == SelectionVertex;
}


bool MeshSelection::isEdge() const
{
    return type() == SelectionEdge;
}


Mesh::Vertex MeshSelection::vertex() const
{
    assert(isVertex());
    return Mesh::Vertex(m_index);
}


Mesh::Edge MeshSelection::edge() const
{
    assert(isEdge());
    return Mesh::Edge(m_index);
}



Document::Document(QObject *parent)
    : QObject(parent), m_fvSolver(&m_solvedMesh), m_undoStack(new QUndoStack(this))
{

}


const Document::BoundingBox& Document::boundingBox() const
{
    return m_bb;
}


void Document::updateBoundingBox()
{
    typedef Mesh::VertexIterator VertexIterator;

    m_bb.setEmpty();
    for(VertexIterator vit = m_mesh.verticesBegin();
        vit != m_mesh.verticesEnd(); ++vit)
        m_bb.extend(m_mesh.position(*vit));
}


MeshSelection Document::selection() const
{
    return m_selection;
}


void Document::setSelection(MeshSelection selection)
{
    assert(isSelectionValid(selection));
    if(selection != m_selection)
    {
        m_selection = selection;
        emit selectionChanged();
    }
}


bool Document::isSelectionValid(MeshSelection selection)
{
    return selection.isNone()                                          ||
          (selection.isVertex() && mesh().isValid(selection.vertex())) ||
          (selection.isEdge()   && mesh().isValid(selection.edge()));
}


float Document::vertexSqrDist(Mesh::Vertex v, const Eigen::Vector2f& p) const
{
    Eigen::Vector2f vp = m_mesh.position(v);
    return (vp - p).squaredNorm();
}


Mesh::Vertex Document::closestVertex(const Eigen::Vector2f& p,
                                     float* sqrDist) const
{
    Mesh::VertexIterator vit = m_mesh.verticesBegin();
    Mesh::Vertex closest = *vit;
    float dist = vertexSqrDist(*vit, p);

    for(++vit; vit != m_mesh.verticesEnd(); ++vit)
    {
        float vdist = vertexSqrDist(*vit, p);
        if(vdist < dist)
        {
            dist = vdist;
            closest = *vit;
        }
    }

    if(sqrDist) *sqrDist = dist;
    return closest;
}


float Document::edgeSqrDist(Mesh::Edge e, const Eigen::Vector2f& p) const
{
    Eigen::Vector2f p0 = m_mesh.position(m_mesh.vertex(e, 0));
    Eigen::Vector2f p1 = m_mesh.position(m_mesh.vertex(e, 1));

    Eigen::Vector2f edge = p1 - p0;

    Eigen::Vector2f v0 = p - p0;
    float d0 = edge.dot(v0);
    if(d0 <= 0)
        return v0.squaredNorm();

    Eigen::Vector2f v1 = p - p1;
    if(edge.dot(v1) >= 0)
        return v1.squaredNorm();

    return v0.squaredNorm() - d0*d0 / edge.squaredNorm();
}


Document::Mesh::Edge Document::closestEdge(const Eigen::Vector2f& p,
                                           float *sqrDist) const
{
    Mesh::EdgeIterator eit = m_mesh.edgesBegin();
    Mesh::Edge closest = *eit;
    float dist = edgeSqrDist(*eit, p);

    for(++eit; eit != m_mesh.edgesEnd(); ++eit)
    {
        float edist = edgeSqrDist(*eit, p);
        if(edist < dist)
        {
            dist = edist;
            closest = *eit;
        }
    }

    if(sqrDist) *sqrDist = dist;
    return closest;
}


void Document::solve()
{
    m_solvedMesh = m_mesh;
    m_solvedMesh.finalize();
    // Would break undo
    //m_solvedMesh.compactNodes();

    m_fvSolver.build();
    m_fvSolver.sort();
    m_fvSolver.solve();

    emit meshUpdated();
}


Document::Mesh& Document::mesh()
{
    return m_mesh;
}


const Document::Mesh& Document::mesh() const
{
    return m_mesh;
}


Document::Mesh& Document::solvedMesh()
{
    return m_solvedMesh;
}


const Document::Mesh& Document::solvedMesh() const
{
    return m_solvedMesh;
}


QUndoStack* Document::undoStack()
{
    return m_undoStack;
}


Document::HalfedgeNode Document::swapNode(HalfedgeNode nid) const
{
    if(nid < MaxVertexNode)
        return HalfedgeNode(int(nid) ^ 1);
    return nid;
}


Document::Mesh::Node Document::meshNode(Mesh::Halfedge h, HalfedgeNode nid) const
{
    switch(nid)
    {
    case FromValueNode:
        if(m_mesh.hasVertexFromValue())
            return m_mesh.vertexFromValueNode(h);
        return m_mesh.vertexValueNode(m_mesh.prevHalfedge(h));
    case ToValueNode:   return m_mesh.vertexValueNode(h);
    case EdgeValueNode: return m_mesh.edgeValueNode(h);
    case EdgeGradientNode: return m_mesh.edgeGradientNode(h);
    default: abort();
    }
    return Mesh::Node();
}


Document::Mesh::Node& Document::meshNode(Mesh::Halfedge h, HalfedgeNode nid)
{
    switch(nid)
    {
    case FromValueNode:
        if(m_mesh.hasVertexFromValue())
            return m_mesh.vertexFromValueNode(h);
        return m_mesh.vertexValueNode(m_mesh.prevHalfedge(h));
    case ToValueNode:   return m_mesh.vertexValueNode(h);
    case EdgeValueNode: return m_mesh.edgeValueNode(h);
    case EdgeGradientNode: return m_mesh.edgeGradientNode(h);
    default: abort();
    }
}


void Document::splitNode(Mesh::Halfedge h, HalfedgeNode nid)
{
    Mesh::Node n = meshNode(h, nid);

    Mesh::Halfedge oh = m_mesh.oppositeHalfedge(h);
    HalfedgeNode onid = swapNode(nid);
    Mesh::Node on = meshNode(oh, onid);

    assert(n == on);

    if(!n.isValid())
        return;

    // TODO: make this an UndoCommand
    Mesh::Node nn = m_mesh.addNode(m_mesh.nodeValue(n));

    undoStack()->push(new SetNode(this, oh, onid, nn));
}


void Document::mergeNode(Mesh::Halfedge h, HalfedgeNode nid)
{
    Mesh::Node n = meshNode(h, nid);

    Mesh::Halfedge oh = m_mesh.oppositeHalfedge(h);
    HalfedgeNode onid = swapNode(nid);
    Mesh::Node on = meshNode(oh, onid);

    assert(n != on);

    undoStack()->push(new SetNode(this, oh, onid, n));
}


void Document::setNodeValue(Mesh::Halfedge h, HalfedgeNode nid,
                            const Mesh::NodeValue& value, bool allowMerge)
{
    Mesh::Node n = meshNode(h, nid);

    if(!n.isValid())
    {
        Mesh::Halfedge oh = m_mesh.oppositeHalfedge(h);
        HalfedgeNode onid = swapNode(nid);
        Mesh::Node on = meshNode(oh, onid);

        Mesh::Node nn = m_mesh.addNode(value);

        undoStack()->beginMacro("set node");
        undoStack()->push(new SetNode(this, h, nid, nn));
        if(!on.isValid())
            undoStack()->push(new SetNode(this, oh, onid, nn));
        undoStack()->endMacro();
    }
    else
        undoStack()->push(new SetNodeValue(this, n, value, allowMerge));
}


void Document::loadMesh(const std::string& filename)
{
    Vitelotte::readMvgFromFile(filename, m_mesh);
    m_mesh.setAttributes(Mesh::FV);

    updateBoundingBox();
    setSelection(MeshSelection());

    solve();
}


void Document::openLoadMeshDialog()
{
    QString file = QFileDialog::getOpenFileName(0, "Open mesh", "",
        "Mesh-based vector graphics (*.mvg)");

    if(!file.isEmpty())
    {
        loadMesh(file.toStdString());
    }
}


void Document::openSaveSourceMeshDialog()
{
    QString file = QFileDialog::getSaveFileName(0, "Open mesh", "",
        "Mesh-based vector graphics (*.mvg)");

    if(!file.isEmpty())
    {
        Vitelotte::writeMvgToFile(file.toStdString(), m_mesh);
    }
}


void Document::openSaveFinalMeshDialog()
{
    QString file = QFileDialog::getSaveFileName(0, "Open mesh", "",
        "Mesh-based vector graphics (*.mvg)");

    if(!file.isEmpty())
    {
        Vitelotte::writeMvgToFile(file.toStdString(), m_solvedMesh);
    }
}


SetNodeValue::SetNodeValue(Document *doc, Node node, const NodeValue& value,
                           bool allowMerge)
    : m_document(doc), m_node(node), m_newValue(value),
      m_prevValue(m_document->mesh().nodeValue(m_node)),
      m_allowMerge(allowMerge)
{
}


void SetNodeValue::undo()
{
    m_document->mesh().nodeValue(m_node) = m_prevValue;
    m_document->solve();
}


void SetNodeValue::redo()
{
    m_document->mesh().nodeValue(m_node) = m_newValue;
    m_document->solve();
}


int SetNodeValue::id() const
{
    return 0;
}


bool SetNodeValue::mergeWith(const QUndoCommand* command)
{
    if(command->id() != id())
        return false;
    const SetNodeValue* c = static_cast<const SetNodeValue*>(command);

    if(!c->m_allowMerge || c->m_document != m_document || c->m_node != m_node)
        return false;

    m_newValue = c->m_newValue;

    return true;
}


SetNode::SetNode(Document* doc, Halfedge halfedge,
                 Document::HalfedgeNode nid, Node node)
    : m_document(doc), m_halfedge(halfedge), m_nid(nid), m_newNode(node),
      m_prevNode(m_document->meshNode(m_halfedge, m_nid))
{
}

void SetNode::undo()
{
    m_document->meshNode(m_halfedge, m_nid) = m_prevNode;
    m_document->solve();
}


void SetNode::redo(){
    m_document->meshNode(m_halfedge, m_nid) = m_newNode;
    m_document->solve();
}
