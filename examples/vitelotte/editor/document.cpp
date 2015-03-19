#include <limits>

#include <QFileDialog>
#include <QTime>

#include "Patate/vitelotte_io.h"

#include "../common/vgMeshWithCurvesReader.h"
#include "../common/plotObj.h"

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


MeshSelection::MeshSelection(Mesh::PointConstraint pc)
    : m_type(pc.isValid()? SelectionPointConstraint: SelectionNone),
      m_index(pc.idx())
{}


MeshSelection::MeshSelection(Mesh::Curve c)
    : m_type(c.isValid()? SelectionCurve: SelectionNone),
      m_index(c.idx())
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


bool MeshSelection::isPointConstraint() const
{
    return type() == SelectionPointConstraint;
}


bool MeshSelection::isCurve() const
{
    return type() == SelectionCurve;
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


Mesh::PointConstraint MeshSelection::pointConstraint() const
{
    assert(isPointConstraint());
    return Mesh::PointConstraint(m_index);
}


Mesh::Curve MeshSelection::curve() const
{
    assert(isCurve());
    return Mesh::Curve(m_index);
}


// ////////////////////////////////////////////////////////////////////////////

Document::Document(QObject *parent)
    : QObject(parent),
      m_mesh(),
      m_finalizedMesh(),
      m_solvedMesh(),
      m_bb(),
      m_selection(),
      m_fvSolver(&m_solvedMesh),
      m_undoStack(new QUndoStack(this))
{
    connect(m_undoStack, SIGNAL(indexChanged(int)),
            this, SLOT(solve()));
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
    return selection.isNone()                                                            ||
          (selection.isVertex()          && mesh().isValid(selection.vertex()))          ||
          (selection.isEdge()            && mesh().isValid(selection.edge()))            ||
          (selection.isPointConstraint() && mesh().isValid(selection.pointConstraint()))  ||
          (selection.isCurve()           && mesh().isValid(selection.curve()));
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


Mesh::PointConstraint Document::closestPointConstraint(
        const Eigen::Vector2f& p, float* sqrDist) const
{
    Mesh::PointConstraint closest;
    float dist = std::numeric_limits<float>::infinity();

    for(unsigned pci = 0; pci < m_mesh.nPointConstraints(); ++pci)
    {
        Mesh::PointConstraint pc(pci);
        Mesh::Vertex vx = m_mesh.vertex(pc);
        float vdist = vertexSqrDist(vx, p);
        if(vdist < dist)
        {
            dist = vdist;
            closest = pc;
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


Document::Mesh::Curve Document::closestCurve(const Eigen::Vector2f& p,
                                            float *sqrDist) const
{
    Mesh::Curve closest;
    float dist = std::numeric_limits<float>::infinity();

    for(unsigned ci = 0; ci < m_mesh.nCurves(); ++ci)
    {
        Mesh::Curve curve(ci);
        Mesh::Halfedge h = m_mesh.firstHalfedge(curve);
        while(m_mesh.isValid(h))
        {
            float edist = edgeSqrDist(m_mesh.edge(h), p);
            if(edist < dist)
            {
                dist = edist;
                closest = curve;
            }

            h = m_mesh.nextCurveHalfedge(h);
        }
    }

    if(sqrDist) *sqrDist = dist;
    return closest;
}


void Document::markDirty(unsigned flags)
{
    unsigned currentLevel = m_dirtyFlags & DIRTY_LEVEL_MASK;
    unsigned newLevel     =        flags & DIRTY_LEVEL_MASK;
    unsigned newFlags = (m_dirtyFlags | flags) & ~DIRTY_LEVEL_MASK;

    m_dirtyFlags = std::max(currentLevel, newLevel) | newFlags;
}


Document::Mesh& Document::getMesh(MeshType type)
{
    switch(type)
    {
    case BASE_MESH:
        return m_mesh;
    case FINALIZED_MESH:
        return m_finalizedMesh;
    case SOLVED_MESH:
        return m_solvedMesh;
    }
    abort();
}


const Document::Mesh& Document::getMesh(MeshType type) const
{
    return const_cast<Document*>(this)->getMesh(type);
}


Document::Mesh& Document::mesh()
{
    return m_mesh;
}


const Document::Mesh& Document::mesh() const
{
    return m_mesh;
}


Document::Mesh& Document::finalizedMesh()
{
    return m_finalizedMesh;
}


const Document::Mesh& Document::finalizedMesh() const
{
    return m_finalizedMesh;
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


void Document::splitNode(Mesh::Halfedge h, Mesh::HalfedgeAttribute nid)
{
    Mesh::Node n = m_mesh.halfedgeNode(h, nid);

    Mesh::Halfedge oh = m_mesh.oppositeHalfedge(h);
    Mesh::HalfedgeAttribute onid = m_mesh.oppositeAttribute(nid);

    assert(n == m_mesh.halfedgeNode(oh, onid));

    if(!n.isValid())
        return;

    // TODO: make this an UndoCommand
    Mesh::Node nn = m_mesh.addNode(m_mesh.value(n));

    undoStack()->push(new SetNode(this, oh, onid, nn));
}


void Document::mergeNode(Mesh::Halfedge h, Mesh::HalfedgeAttribute nid)
{
    Mesh::Node n = m_mesh.halfedgeNode(h, nid);

    Mesh::Halfedge oh = m_mesh.oppositeHalfedge(h);
    Mesh::HalfedgeAttribute onid = m_mesh.oppositeAttribute(nid);

    assert(n != m_mesh.halfedgeNode(oh, onid));

    undoStack()->push(new SetNode(this, oh, onid, n));
}


void Document::setNodeValue(Mesh::Halfedge h, Mesh::HalfedgeAttribute nid,
                            const Mesh::Value& value, bool allowMerge)
{
    Mesh::Node n = m_mesh.halfedgeNode(h, nid);

    if(!n.isValid())
    {
        Mesh::Halfedge oh = m_mesh.oppositeHalfedge(h);
        Mesh::HalfedgeAttribute onid = m_mesh.oppositeAttribute(nid);
        Mesh::Node on = m_mesh.halfedgeNode(oh, onid);

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


void Document::solve()
{
    if(m_dirtyFlags == CLEAN)
        return;

    QTime time;
    time.start();
    if(m_dirtyFlags & DIRTY_CURVES_FLAG) {
        std::cout << "Set nodes from curves...  " << std::flush;
        m_mesh.setNodesFromCurves();
        std::cout << time.restart() << " ms" << std::endl;
    }

    std::cout << "Finalize...               " << std::flush;
    m_finalizedMesh = m_mesh;
    m_finalizedMesh.finalize();
    m_finalizedMesh.deleteUnusedNodes();
    m_finalizedMesh.garbageCollection();
    if(connectivityChanged())
        markDirty(DIRTY_CONNECTIVITY);

    m_solvedMesh = m_finalizedMesh;

    std::cout << time.restart() << " ms" << std::endl;
    if(m_solvedMesh.hasUnknowns())
    {
        unsigned dirtyLevel = m_dirtyFlags & DIRTY_LEVEL_MASK;

        // TODO: m_fvSolver.clearErrors();
        switch(dirtyLevel)
        {
        case DIRTY_CONNECTIVITY:
            std::cout << "Build matrix...           " << std::flush;
            m_fvSolver.build();
            std::cout << time.restart() << " ms" << std::endl;
        case DIRTY_NODE_TYPE:
            std::cout << "Sort matrix...            " << std::flush;
            m_fvSolver.sort();
            std::cout << time.restart() << " ms" << std::endl;
            std::cout << "Factorize matrix...       " << std::flush;
            m_fvSolver.factorize();
            std::cout << time.restart() << " ms" << std::endl;
        case DIRTY_NODE_VALUE:
            std::cout << "Solve...                  " << std::flush;
            m_fvSolver.solve();
            std::cout << time.restart() << " ms" << std::endl;
        }

        if(m_fvSolver.error().status() == Vitelotte::SolverError::STATUS_WARNING)
        {
            std::cout << "Solver warning: " << m_fvSolver.error().message() << "\n";
        }
        else if(m_fvSolver.error().status() == Vitelotte::SolverError::STATUS_ERROR)
        {
            std::cout << "Solver error: " << m_fvSolver.error().message() << "\n";
        }
    }
    m_dirtyFlags = CLEAN;

    emit meshUpdated();
}


void Document::loadMesh(const std::string& filename)
{
    VGMeshWithCurveReader reader;
    std::ifstream in(filename.c_str());
    reader.read(in, m_mesh);
    m_mesh.setAttributes(Mesh::FV_FLAGS);

    updateBoundingBox();
    setSelection(MeshSelection());

    unsigned dirtyCurves = (m_mesh.nCurves() || m_mesh.nPointConstraints())?
                DIRTY_CURVES_FLAG: 0;
    markDirty(DIRTY_CONNECTIVITY | dirtyCurves);
    solve();

    emit meshChanged();
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


void Document::exportPlot(const std::string& filename, unsigned layer)
{
    typedef Vitelotte::FVElement<float> FVElem;

    ::exportPlot<Mesh, FVElem>(m_solvedMesh, filename, layer, 4);

//    unsigned nCount = 0;
//    std::vector<int> indexFromNode(m_solvedMesh.nNodes(), -1);
//    for(Mesh::FaceIterator fit = m_solvedMesh.facesBegin();
//        fit != m_solvedMesh.facesEnd(); ++fit)
//    {
//        Mesh::Vector pts[3];
//        Eigen::Matrix<float, 4, 6> values;

//        Mesh::Halfedge h[3];
//        h[0] = m_solvedMesh.halfedge(*fit);
//        h[1] = m_solvedMesh.nextHalfedge(h[0]);
//        h[2] = m_solvedMesh.nextHalfedge(h[1]);

//        for(unsigned hi = 0; hi < 3; ++hi)
//        {
//            Mesh::Node n;
//            Mesh::Vertex to = m_solvedMesh.toVertex(h[hi]);
//            Mesh::Vertex from = m_solvedMesh.fromVertex(h[hi]);

//            n = m_solvedMesh.toVertexValueNode(h[hi]);
//            if(indexFromNode[n.idx()] == -1)
//            {
//                out << "v " << m_solvedMesh.position(to).transpose() << " ";
//                if(m_solvedMesh.isValid(n))
//                    out << m_solvedMesh.value(n)(layer);
//                else
//                    out << "0";
//                out << "\n";
//                indexFromNode[n.idx()] = nCount++;
//            }

//            n = m_solvedMesh.edgeValueNode(h[hi]);
//            if(indexFromNode[n.idx()] == -1)
//            {
//                Mesh::Vector p = (m_solvedMesh.position(to)
//                                + m_solvedMesh.position(from)) / 2;
//                out << "v " << p.transpose() << " ";
//                if(m_solvedMesh.isValid(n))
//                    out << m_solvedMesh.value(n)(layer);
//                else
//                    out << "0";
//                out << "\n";
//                indexFromNode[n.idx()] = nCount++;
//            }
//        }
//    }

//    for(Mesh::FaceIterator fit = m_solvedMesh.facesBegin();
//        fit != m_solvedMesh.facesEnd(); ++fit)
//    {
//        Mesh::Halfedge h[3];
//        h[0] = m_solvedMesh.halfedge(*fit);
//        h[1] = m_solvedMesh.nextHalfedge(h[0]);
//        h[2] = m_solvedMesh.nextHalfedge(h[1]);

//        unsigned vi[3];
//        unsigned ei[3];
//        for(int i = 0; i < 3; ++i)
//        {
//            vi[i] = indexFromNode[m_solvedMesh.toVertexValueNode(h[i]).idx()];
//            ei[(i+1)%3] = indexFromNode[m_solvedMesh.edgeValueNode(h[i]).idx()];
//        }

////        out << "f " << vi[0]+1 << " " << vi[1]+1 << " " << vi[2]+1 << "\n";
//        out << "f " << vi[0]+1 << " " << ei[2]+1 << " " << ei[1]+1 << "\n";
//        out << "f " << vi[1]+1 << " " << ei[0]+1 << " " << ei[2]+1 << "\n";
//        out << "f " << vi[2]+1 << " " << ei[1]+1 << " " << ei[0]+1 << "\n";
//        out << "f " << ei[0]+1 << " " << ei[1]+1 << " " << ei[2]+1 << "\n";
//    }
}


bool Document::connectivityChanged() const
{
    Mesh::HalfedgeIterator fhit  = m_finalizedMesh.halfedgesBegin();
    Mesh::HalfedgeIterator fhend = m_finalizedMesh.halfedgesEnd();
    Mesh::HalfedgeIterator shit  =    m_solvedMesh.halfedgesBegin();
    Mesh::HalfedgeIterator shend =    m_solvedMesh.halfedgesEnd();

    for(; fhit != fhend && shit != shend; ++fhit, ++shit)
    {
        for(unsigned i = 0; i < 4; ++i)
        {
            if(m_finalizedMesh.halfedgeNode(*fhit, Mesh::HalfedgeAttribute(i)) !=
                    m_solvedMesh.halfedgeNode(*shit, Mesh::HalfedgeAttribute(i)))
                return true;
        }
    }
    return false;
}


// ////////////////////////////////////////////////////////////////////////////

SetNodeValue::SetNodeValue(Document *doc, Node node, const Value& value,
                           bool allowMerge)
    : m_document(doc), m_node(node), m_newValue(value),
      m_prevValue(m_document->mesh().value(m_node)),
      m_allowMerge(allowMerge)
{
}


void SetNodeValue::undo()
{
    m_document->mesh().value(m_node) = m_prevValue;

    bool prevConstrained = (m_prevValue != m_document->mesh().unconstrainedValue());
    bool newConstrained  = ( m_newValue != m_document->mesh().unconstrainedValue());
    m_document->markDirty((prevConstrained != newConstrained)?
        Document::DIRTY_NODE_TYPE: Document::DIRTY_NODE_VALUE);
}


void SetNodeValue::redo()
{
    m_document->mesh().value(m_node) = m_newValue;

    bool prevConstrained = (m_prevValue != m_document->mesh().unconstrainedValue());
    bool newConstrained  = ( m_newValue != m_document->mesh().unconstrainedValue());
    m_document->markDirty((prevConstrained != newConstrained)?
        Document::DIRTY_NODE_TYPE: Document::DIRTY_NODE_VALUE);
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


// ////////////////////////////////////////////////////////////////////////////

SetNode::SetNode(Document* doc, Halfedge halfedge,
                 Mesh::HalfedgeAttribute nid, Node node)
    : m_document(doc), m_halfedge(halfedge), m_nid(nid), m_newNode(node),
      m_prevNode(m_document->mesh().halfedgeNode(m_halfedge, m_nid))
{
}

void SetNode::undo()
{
    m_document->mesh().halfedgeNode(m_halfedge, m_nid) = m_prevNode;
    m_document->markDirty(Document::DIRTY_CONNECTIVITY);
}


void SetNode::redo(){
    m_document->mesh().halfedgeNode(m_halfedge, m_nid) = m_newNode;
    m_document->markDirty(Document::DIRTY_CONNECTIVITY);
}


// ////////////////////////////////////////////////////////////////////////////

MoveGradientStop::MoveGradientStop(
        Document* doc, Curve curve, unsigned which, Scalar fromPos, Scalar toPos,
        bool allowMerge)
    : m_document(doc), m_curve(curve), m_which(which),
      m_fromPos(fromPos), m_toPos(toPos), m_allowMerge(allowMerge)
{
}

void MoveGradientStop::undo()
{
    move(m_toPos, m_fromPos);
}

void MoveGradientStop::redo()
{
    move(m_fromPos, m_toPos);
}

int MoveGradientStop::id() const
{
    return 1;
}

bool MoveGradientStop::mergeWith(const QUndoCommand* command)
{
    if(command->id() != id())
        return false;
    const MoveGradientStop* mgs = static_cast<const MoveGradientStop*>(command);

    if(!mgs->m_allowMerge || mgs->m_document != m_document || mgs->m_curve != m_curve ||
            mgs->m_fromPos != m_toPos)
        return false;

    m_toPos = mgs->m_toPos;

    return true;
}


void MoveGradientStop::move(Scalar from, Scalar to)
{
    Mesh& mesh = m_document->mesh();
    Mesh::ValueFunction& grad = mesh.valueFunction(m_curve, m_which);
    Mesh::Value color = grad.sample(from);
    grad.remove(from);
    grad.add(to, color);

    m_document->markDirty(Document::DIRTY_NODE_VALUE | Document::DIRTY_CURVES_FLAG);
}


// ////////////////////////////////////////////////////////////////////////////

SetGradientStopValue::SetGradientStopValue(
        Document* doc, Curve curve, unsigned which,
        Scalar pos, const Value& value)
    : m_document(doc), m_curve(curve), m_which(which), m_pos(pos),
      m_prevValue(doc->mesh().valueFunction(curve, which).sample(pos)),
      m_newValue(value)
{
}

void SetGradientStopValue::undo()
{
    setColor(m_prevValue);
}


void SetGradientStopValue::redo()
{
    setColor(m_newValue);
}

void SetGradientStopValue::setColor(const Value& color)
{
    m_document->mesh().valueFunction(m_curve, m_which).sample(m_pos) = color;
    m_document->markDirty(Document::DIRTY_NODE_VALUE | Document::DIRTY_CURVES_FLAG);
}


// ////////////////////////////////////////////////////////////////////////////

AddRemoveGradientStop::AddRemoveGradientStop(
        Document* doc, Curve curve, unsigned which,
        Scalar pos, const Value& value)
    : m_document(doc), m_curve(curve), m_which(which),
      m_pos(pos), m_value(value), m_add(true)
{
}

AddRemoveGradientStop::AddRemoveGradientStop(
        Document* doc, Curve curve, unsigned which,
        Scalar pos)
    : m_document(doc), m_curve(curve), m_which(which),
      m_pos(pos), m_value(), m_add(false)
{
    Mesh& mesh = m_document->mesh();
    Mesh::ValueFunction& grad = mesh.valueFunction(m_curve, m_which);
    m_value = grad.sample(m_pos);
}

void AddRemoveGradientStop::undo()
{
    if(m_add) remove();
    else      add();
}

void AddRemoveGradientStop::redo()
{
    if(m_add) add();
    else      remove();
}

void AddRemoveGradientStop::add()
{
    Mesh& mesh = m_document->mesh();
    Mesh::ValueFunction& grad = mesh.valueFunction(m_curve, m_which);
    grad.add(m_pos, m_value);

    unsigned dFlag = (grad.size() == 1)? Document::DIRTY_NODE_TYPE:
                                         Document::DIRTY_NODE_VALUE;
    m_document->markDirty(dFlag | Document::DIRTY_CURVES_FLAG);
}

void AddRemoveGradientStop::remove()
{
    Mesh& mesh = m_document->mesh();
    Mesh::ValueFunction& grad = mesh.valueFunction(m_curve, m_which);
    grad.remove(m_pos);

    unsigned dFlag = (grad.size() == 0)? Document::DIRTY_NODE_TYPE:
                                         Document::DIRTY_NODE_VALUE;
    m_document->markDirty(dFlag | Document::DIRTY_CURVES_FLAG);
}


// ////////////////////////////////////////////////////////////////////////////

SetGradient::SetGradient(Document* doc, Curve curve, unsigned which,
            const ValueFunction& grad)
    : m_document(doc),
      m_curve(curve),
      m_which(which),
      m_prevGrad(m_document->mesh().valueFunctionRaw(m_curve, m_which)),
      m_newGrad(grad)
{
}

void SetGradient::undo()
{
    setGradient(m_prevGrad);
}

void SetGradient::redo()
{
    setGradient(m_newGrad);
}

void SetGradient::setGradient(const ValueFunction& grad)
{
    ValueFunction& cgrad = m_document->mesh().valueFunctionRaw(m_curve, m_which);
    bool prevFree = cgrad.empty();
    bool newFree = grad.empty();
    cgrad = grad;

    unsigned dFlag = (prevFree == newFree)? Document::DIRTY_NODE_VALUE:
                                            Document::DIRTY_NODE_TYPE;
    m_document->markDirty(dFlag | Document::DIRTY_CURVES_FLAG);
}


// ////////////////////////////////////////////////////////////////////////////

SetPointConstraintValue::SetPointConstraintValue(Document* doc, PointConstraint pc,
                                       const Value& value)
    : m_document(doc), m_pc(pc),
      m_prevValue(doc->mesh().value(m_pc)),
      m_newValue(value)
{
}

void SetPointConstraintValue::undo()
{
    setValue(m_prevValue);
}


void SetPointConstraintValue::redo()
{
    setValue(m_newValue);
}

void SetPointConstraintValue::setValue(const Value& value)
{
    m_document->mesh().value(m_pc) = value;
    m_document->markDirty(Document::DIRTY_NODE_VALUE | Document::DIRTY_CURVES_FLAG);
}


// ////////////////////////////////////////////////////////////////////////////

SetPointConstraintGradient::SetPointConstraintGradient(Document* doc, PointConstraint pc,
                                       const NodeGradient& gradient)
    : m_document(doc), m_pc(pc),
      m_prevGradient(doc->mesh().gradient(m_pc)),
      m_newGradient(gradient)
{
}

void SetPointConstraintGradient::undo()
{
    setGradient(m_prevGradient);
}


void SetPointConstraintGradient::redo()
{
    setGradient(m_newGradient);
}

void SetPointConstraintGradient::setGradient(const NodeGradient& gradient)
{
    m_document->mesh().gradient(m_pc) = gradient;
    m_document->markDirty(Document::DIRTY_CONNECTIVITY | Document::DIRTY_CURVES_FLAG);
}


// ////////////////////////////////////////////////////////////////////////////


SetCurveFlags::SetCurveFlags(Document* doc, Curve curve,
                  unsigned flags)
    : m_document(doc),
      m_curve(curve),
      m_prevFlags(m_document->mesh().flags(curve)),
      m_newFlags(flags)
{
}


void SetCurveFlags::undo()
{
    setFlags(m_prevFlags);
}


void SetCurveFlags::redo()
{
    setFlags(m_newFlags);
}


void SetCurveFlags::setFlags(unsigned flags)
{
    m_document->mesh().setFlagsRaw(m_curve, flags);
    m_document->markDirty(Document::DIRTY_CONNECTIVITY | Document::DIRTY_CURVES_FLAG);
}
