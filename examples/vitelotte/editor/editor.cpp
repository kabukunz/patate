#include <cstdlib>
#include <iostream>

//#include <Eigen/Geometry>

#include <QMouseEvent>
#include <QWheelEvent>

#include "document.h"

#include "editor.h"


Editor::Editor(QWidget* parent)
    : QGLWidget(parent),
      m_document(0),
      m_initialized(false),
      m_showConstraints(true),
      m_showWireframe(false),
      m_nodeMeshType(Document::BASE_MESH),
      m_editMode(EDIT_NODES),
      m_camera(),
      m_inputState(STATE_IDLE),
      m_dragPos(),
      m_dragGradientStop(),
      m_paintColor(0., 0., 0., 1.),
      m_rendererDirty(true)
{
    setMouseTracking(true);
}


Editor::~Editor()
{
}


Eigen::Vector2f Editor::sceneFromView(const QPointF& view) const
{
    return m_camera.normalizedToCamera(screenToNormalized(view));
}


Eigen::Vector2f Editor::screenToNormalized(const QPointF& screen) const
{
    return Eigen::Vector2f(
                screen.x() * 2.f / width() - 1.f,
                (height()-screen.y()) * 2.f / height() - 1.f);
}


QPointF Editor::normalizedToScreen(const Eigen::Vector2f& normalized) const
{
    return QPointF(
            (normalized.x() + 1.f) *  width() / 2.f,
            (normalized.y() + 1.f) * height() / 2.f);
}


void Editor::centerView()
{
    assert(m_document);

    const Document::BoundingBox& bb = m_document->boundingBox();
    Eigen::Vector3f center;
    center << bb.center(), 0;
    Eigen::Vector3f scale;
    scale(0) = bb.sizes().maxCoeff() * .6;
    scale(1) = scale(0);
    scale(2) = 1;
    m_camera.setViewBox(OrthographicCamera::ViewBox(
        center - scale,
        center + scale));
    m_camera.changeAspectRatio(float(width()) / float(height()));

    update();
}


void Editor::setDocument(Document* document)
{
    if(document == m_document)
        return;

    if(m_document)
    {
        disconnect(m_document);
        m_document->disconnect(this);
    }

    m_document = document;

    if(m_document)
    {
        centerView();

        if(m_initialized)
            updateBuffers();

        connect(m_document, SIGNAL(meshChanged()), this, SLOT(centerView()));
        connect(m_document, SIGNAL(meshUpdated()), this, SLOT(updateBuffers()));
        connect(m_document, SIGNAL(meshUpdated()), this, SLOT(updateRenderers()));
        connect(m_document, SIGNAL(selectionChanged()),
                this, SLOT(updateRenderers()));
    }
}


void Editor::updateBuffers()
{
    assert(m_document);
    m_renderer.updateBuffers(mesh());
    update();
}


void Editor::updateRenderers()
{
    m_rendererDirty = true;
    update();
}


void Editor::setShowConstraints(bool enable)
{
    if(enable != m_showConstraints)
    {
        m_showConstraints = enable;
        updateRenderers();
    }
}


void Editor::setShowWireframe(bool enable)
{
    if(enable != m_showWireframe)
    {
        m_showWireframe = enable;
        updateRenderers();
    }
}


void Editor::setShowMesh(int type)
{
    m_nodeMeshType = Document::MeshType(type);
    update();
}


void Editor::setEditMode(int mode)
{
    m_editMode = EditMode(mode);
}


void Editor::setPaintColor(const QColor& color) {
    m_paintColor = NodeValue(color.redF(), color.greenF(),
                             color.blueF(), color.alphaF());
}


void Editor::initializeGL()
{
    std::cout << "OpenGL Vendor:       " << glGetString(GL_VENDOR) << "\n";
    std::cout << "OpenGL Renderer:     " << glGetString(GL_RENDERER) << "\n";
    std::cout << "OpenGL Version:      " << glGetString(GL_VERSION) << "\n";
    std::cout << "OpenGL GLSL Version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << "\n";
    if(format().sampleBuffers())
        std::cout << "OpenGL Multisample:  " << format().samples() << "\n";

    GLenum res = glewInit();
    if (res != GLEW_OK)
    {
        std::cerr << "GLEW initialization error: '" << glewGetErrorString(res) <<"'\n";
        abort();
    }
    else if(!GLEW_VERSION_3_0)
    {
        std::cerr << "OpenGL 3.0 not supported. Aborting.\n";
        abort();
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glClearColor(.5, .5, .5, 1.);

    m_initialized = true;
    m_renderer.initialize();
    if(m_document) {
        m_renderer.updateBuffers(mesh());
        centerView();
    }
}

void Editor::resizeGL(int width, int height)
{
    glViewport(0, 0, width, height);
    m_camera.changeAspectRatio(float(width) / float(height));
}

void Editor::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if(m_document) {
        doUpdateRenderers();

        Eigen::Matrix4f viewMatrix = m_camera.projectionMatrix();

        // TODO: what to do about SRGB ?
        glDisable(GL_FRAMEBUFFER_SRGB);

        m_renderer.render(viewMatrix);

//        m_renderer.renderWireframe(viewMatrix,
//                                   zoom(), .5, Eigen::Vector4f(.5, .5, .5, 1.));

        glEnable(GL_FRAMEBUFFER_SRGB);

        Eigen::Vector2f viewportSize(width(), height());
        m_lineRenderer.render(viewMatrix, viewportSize);
        m_pointRenderer.render(viewMatrix, viewportSize);

        if(m_showWireframe)
        {
            m_nodeRenderer.render(viewMatrix, viewportSize);
        }
    }
}


void Editor::mousePressEvent(QMouseEvent* event)
{
    if(   m_inputState == STATE_IDLE
       && m_editMode == EDIT_CURVES
       && event->button() == Qt::LeftButton
       && m_document->selection().isCurve())
    {
        Vector scenePos = sceneFromView(event->localPos());
        GradientStop* gs = pickGradientStop(scenePos);
        if(gs) beginDragStop(gs);
    }
    else if(   m_inputState == STATE_IDLE
       && event->button() == Qt::MidButton)
    {
        beginPanView(sceneFromView(event->localPos()));
    }
}


void Editor::mouseReleaseEvent(QMouseEvent* event)
{
    switch(m_inputState)
    {
    case STATE_IDLE:
    {
        if(event->button() == Qt::LeftButton)
        {
            Vector scenePos = sceneFromView(event->localPos());
            if(m_editMode == EDIT_NODES)
            {
                pickVertexOrEdge(scenePos);
            }
            else if(m_editMode == EDIT_CURVES)
            {
                if(!trySetPointConstraint(scenePos))
                {
                    GradientStop gs;
                    makeNewStop(gs, scenePos);
                    if(gs.curve.isValid()) {
                        addGradientStop(gs);
                        hideDummyStop();
                    }
                    else
                    {
                        pickConstraint(scenePos);
                    }
                }
            }
        }
        else if(   event->button() == Qt::RightButton
                && m_editMode == EDIT_CURVES)
        {
            GradientStop* gs = pickGradientStop(sceneFromView(event->localPos()));
            if(gs) removeGradientStop(*gs);
        }
        break;
    }
    case STATE_PAN_VIEW:
        if(event->button() != Qt::MidButton)
            break;
        endPanView();
        break;
    case STATE_GRABED_GRADIENT_STOP:
        if(event->button() != Qt::LeftButton)
            break;
        setStopColor(m_dragGradientStop);
        endDragStop();
        break;
    case STATE_DRAG_GRADIENT_STOP:
        if(event->button() != Qt::LeftButton)
            break;
        endDragStop();
        break;
    }
}

void Editor::mouseMoveEvent(QMouseEvent* event)
{
    hideDummyStop();

    switch(m_inputState)
    {
    case STATE_IDLE:
        if(m_editMode == EDIT_CURVES)
        {
            Vector scenePos = sceneFromView(event->localPos());
            GradientStop* gs = pickGradientStop(scenePos);
            if(gs) selectGradientStop(gs);
            else   showDummyStop(scenePos);
        }
        break;
    case STATE_PAN_VIEW:
        panView(sceneFromView(event->localPos()));
        break;
    case STATE_GRABED_GRADIENT_STOP:
    case STATE_DRAG_GRADIENT_STOP:
        dragStop(sceneFromView(event->localPos()));
        break;
    }
}


void Editor::wheelEvent(QWheelEvent* event)
{
    m_camera.normalizedZoom(
                screenToNormalized(event->posF()),
                event->delta() > 0? 1.1: 1/1.1);

    // Force update selection
//    m_currentSelection = MeshSelection();
    updateRenderers();
}


float Editor::zoom() const
{
    return width() / m_camera.getViewBox().sizes()(0);
}


void Editor::beginPanView(const Vector& scenePos)
{
    grabMouse();
    m_inputState = STATE_PAN_VIEW;
    m_dragPos = m_camera.cameraToNormalized(scenePos);
}

void Editor::endPanView()
{
    releaseMouse();
    m_inputState = STATE_IDLE;
}

void Editor::panView(const Vector& scenePos)
{
    Eigen::Vector2f norm = m_camera.cameraToNormalized(scenePos);
    if(norm != m_dragPos)
    {
        m_camera.normalizedTranslate(norm - m_dragPos);
        m_dragPos = norm;
        update();
    }
}


void Editor::pickVertexOrEdge(const Vector& scenePos)
{
    float selDist = 8 * m_camera.getViewBox().sizes()(0) / width();

    float vSqrDist;
    Mesh::Vertex vSel = m_document->closestVertex(scenePos, &vSqrDist);
    if(vSqrDist < selDist*selDist)
    {
        m_document->setSelection(MeshSelection(vSel));
        return;
    }

    float eSqrDist;
    Mesh::Edge eSel = m_document->closestEdge(scenePos, &eSqrDist);
    if(eSqrDist < selDist*selDist)
    {
        m_document->setSelection(MeshSelection(eSel));
        return;
    }

    m_document->setSelection(MeshSelection());
}

void Editor::pickConstraint(const Vector& scenePos)
{
    float selDist = 8 * m_camera.getViewBox().sizes()(0) / width();

    float pcSqrDist;
    Mesh::PointConstraint pcSel =
            m_document->closestPointConstraint(scenePos, &pcSqrDist);
    if(pcSqrDist < selDist*selDist)
    {
        m_document->setSelection(MeshSelection(pcSel));
        return;
    }

    float cSqrDist;
    Mesh::Curve cSel = m_document->closestCurve(scenePos, &cSqrDist);
    if(cSqrDist < selDist*selDist)
    {
        m_document->setSelection(MeshSelection(cSel));
        return;
    }

    m_document->setSelection(MeshSelection());
}


Editor::GradientStop* Editor::pickGradientStop(const Vector& scenePos)
{
    float selDist = 8 * m_camera.getViewBox().sizes()(0) / width();

    float gsSqrDist;
    GradientStop* gs = closestGradientStop(scenePos, &gsSqrDist);

    if(gs && gsSqrDist < selDist*selDist)
        return gs;
    return 0;
}

void Editor::beginDragStop(GradientStop* gs)
{
    grabMouse();
    m_inputState = STATE_GRABED_GRADIENT_STOP;
    m_dragGradientStop = *gs;
}

void Editor::endDragStop()
{
    releaseMouse();
    m_inputState = STATE_IDLE;
    m_dragGradientStop.curve = Mesh::Curve();
}

void Editor::dragStop(const Vector& scenePos)
{
    float newPos = closestPos(m_dragGradientStop.curve, scenePos);

    Mesh& mesh = m_document->mesh();
    const Mesh::ValueGradient& vg = mesh.valueGradient(
                m_dragGradientStop.curve, m_dragGradientStop.which);
    if(vg.count(newPos))
        return;
    m_document->undoStack()->push(new MoveGradientStop(
        m_document, m_dragGradientStop.curve, m_dragGradientStop.which,
        m_dragGradientStop.gpos, newPos,
        m_inputState == STATE_DRAG_GRADIENT_STOP));

    m_inputState = STATE_DRAG_GRADIENT_STOP;
    m_dragGradientStop.gpos = newPos;

    updateRenderers();
}


void Editor::makeNewStop(GradientStop& gs, const Vector& scenePos)
{
    Mesh& mesh = m_document->mesh();

    if(!m_document->selection().isCurve())
        return;

    Mesh::Curve curve = m_document->selection().curve();

    float addDist = 16 / zoom();

    unsigned side;
    float sqrDist;
    float pos = closestPos(curve, scenePos, &side, &sqrDist);
    if(mesh.isValid(curve) && sqrDist < addDist*addDist)
    {
        float offset = mesh.valueTear(curve)? 8: 0;
        gs.curve = curve;
        gs.which = side | Mesh::VALUE;
        gs.gpos = pos;
        gs.position = computeStopPosition(
                curve, pos, (side == Mesh::LEFT)? offset: -offset);
        gs.color = m_paintColor;
    }
}


void Editor::showDummyStop(const Vector& scenePos)
{
    GradientStop gs = m_dummyStop;
    makeNewStop(m_dummyStop, scenePos);
    if(m_dummyStop != gs)
        updateRenderers();
}

void Editor::hideDummyStop()
{
    if(m_dummyStop.curve.isValid())
    {
        m_dummyStop.curve = Mesh::Curve();
        updateRenderers();
    }
}


void Editor::setStopColor(const GradientStop& gs)
{
    m_document->undoStack()->push(new SetGradientStopValue(
        m_document, gs.curve, gs.which, gs.gpos, m_paintColor));
}

void Editor::addGradientStop(const GradientStop& gs)
{
    m_document->undoStack()->push(new AddRemoveGradientStop(
        m_document, gs.curve, gs.which, gs.gpos, m_paintColor));
}

void Editor::removeGradientStop(const GradientStop& gs)
{
    m_document->undoStack()->push(new AddRemoveGradientStop(
        m_document, gs.curve, gs.which, gs.gpos));
}


bool Editor::trySetPointConstraint(const Vector& pos)
{
    if(!m_document->selection().isPointConstraint())
        return false;

    Mesh& mesh = m_document->mesh();

    const Mesh::PointConstraint pc = m_document->selection().pointConstraint();
    const Vector& pcp = mesh.position(mesh.vertex(pc));

    float selDist = 8 / zoom();

    if((pcp - pos).squaredNorm() < selDist*selDist) {
        m_document->undoStack()->push(new SetPointConstraint(
            m_document, pc, m_paintColor));
        return true;
    }
    return false;
}


void Editor::selectGradientStop(GradientStop* gs)
{

}


void Editor::doUpdateRenderers()
{
    assert(m_document);

    Mesh& mesh = m_document->mesh();
    MeshSelection sel = m_document->selection();

    if(!m_rendererDirty)
        return;
    m_rendererDirty = false;

    m_gradientStops.clear();
    m_pointRenderer.clear();
    m_lineRenderer.clear();

    if(m_showConstraints && !m_showWireframe) {
        for(unsigned pci = 0; pci < mesh.nPointConstraints(); ++pci)
        {
            Eigen::Vector3f p;
            p << mesh.position(mesh.vertex(Mesh::PointConstraint(pci))), 0;
            m_pointRenderer.addPoint(p, 3, Eigen::Vector4f(0, 0, 0, 1));
        }
        for(unsigned ci = 0; ci < mesh.nCurves(); ++ci)
            drawCurve(Mesh::Curve(ci), 2, Eigen::Vector4f(0, 0, 0, 1));
    }

    if(m_showWireframe) {
        m_nodeRenderer.update(m_document->getMesh(m_nodeMeshType),
                              zoom());
    }


    if(sel.isVertex() || sel.isPointConstraint())
    {
        Mesh::Vertex v = sel.isVertex()?
                    sel.vertex():
                    mesh.vertex(sel.pointConstraint());
        Eigen::Vector3f p; p << mesh.position(v), 0;

        float innerRadius = 3;
        float outerRadius = innerRadius + 1.5;
        Eigen::Vector4f innerColor(1., 1., 1., 1.);
        Eigen::Vector4f outerColor(0., 0., 0., 1.);

        m_pointRenderer.addPoint(p, outerRadius, outerColor);
        m_pointRenderer.addPoint(p, innerRadius, innerColor);
    }
    else if(sel.isEdge())
    {
        Mesh::Edge e = sel.edge();

        Eigen::Vector3f p0; p0 << mesh.position(mesh.vertex(e, 0)), 0;
        Eigen::Vector3f p1; p1 << mesh.position(mesh.vertex(e, 1)), 0;

        float innerWidth = 1.5;
        float outerWidth = 4.5;
        Eigen::Vector4f innerColor(1., 1., 1., 1.);
        Eigen::Vector4f outerColor(0., 0., 0., 1.);

        m_lineRenderer.addPoint(p0, outerWidth, outerColor);
        m_lineRenderer.addPoint(p1, outerWidth, outerColor);
        m_lineRenderer.endLine();
        m_lineRenderer.addPoint(p0, innerWidth, innerColor);
        m_lineRenderer.addPoint(p1, innerWidth, innerColor);
        m_lineRenderer.endLine();
    }
    else if(sel.isCurve())
    {
        float innerWidth = 1.5;
        float outerWidth = 4.5;
        Eigen::Vector4f innerColor(1., 1., 1., 1.);
        Eigen::Vector4f outerColor(0., 0., 0., 1.);

        Mesh::Curve curve = sel.curve();

        drawCurve(curve, outerWidth, outerColor);
        drawCurve(curve, innerWidth, innerColor);

        float innerRadius = 3;
        float outerRadius = innerRadius + 1.5;

        bool tear = mesh.valueTear(curve);
        Scalar offset = tear? 8: 0;

        addGradientStops(curve, Mesh::VALUE_LEFT, offset);

        if(tear)
            addGradientStops(curve, Mesh::VALUE_RIGHT, -offset);
        drawGradientStops(innerRadius, outerRadius);
    }
    m_pointRenderer.upload();
    m_lineRenderer.upload();
}


Mesh::Vector Editor::computeStopPosition(
        Mesh::Curve curve, float pos, float offset, Mesh::Halfedge* from)
{
    Mesh& mesh = m_document->mesh();

    Mesh::Halfedge h = from? *from: mesh.firstHalfedge(curve);

    if(pos < mesh.fromCurvePos(h))
        pos = mesh.fromCurvePos(h);
    while(mesh.isValid(h) && pos > mesh.toCurvePos(h))
        h = mesh.nextCurveHalfedge(h);
    if(!mesh.isValid(h))
        pos = 1;

    if(from) *from = h;

    Vector p0 = mesh.position(mesh.fromVertex(h));
    Vector p1 = mesh.position(mesh.toVertex(h));
    float pos0 = mesh.fromCurvePos(h);
    float pos1 = mesh.toCurvePos(h);

    Vector v = p1 - p0;
    Vector n = Vector(-v.y(), v.x()).normalized();
    float alpha = (pos - pos0) / (pos1 - pos0);

    return p0 + v * alpha + n * offset / zoom();
}


void Editor::addGradientStops(Mesh::Curve curve, unsigned which, float offset)
{
    Mesh& mesh = m_document->mesh();

    Mesh::Halfedge h = mesh.firstHalfedge(curve);

    Mesh::ValueGradient::const_iterator sit =
            mesh.valueGradient(curve, which).begin();
    Mesh::ValueGradient::const_iterator send =
            mesh.valueGradient(curve, which).end();
    for(; sit != send; ++sit)
    {
        float pos = sit->first;
        const Mesh::NodeValue& color = sit->second;

        Vector p = computeStopPosition(curve, pos, offset, &h);

        m_gradientStops.push_back(GradientStop(
            curve, which, pos, p, color));
    }
}


Editor::GradientStop* Editor::closestGradientStop(const Vector& p, float* sqrDist)
{
    GradientStop* closest = 0;
    float closestSqrDist = std::numeric_limits<float>::infinity();
    for(GradientStopList::iterator gsit = m_gradientStops.begin();
        gsit != m_gradientStops.end(); ++gsit)
    {
        float dist = (p - gsit->position).squaredNorm();
        if(dist < closestSqrDist)
        {
            closest = &(*gsit);
            closestSqrDist = dist;
        }
    }

    if(sqrDist)
        *sqrDist = closestSqrDist;
    return closest;
}


float Editor::closestPos(Mesh::Curve curve, const Vector& p,
                         unsigned* side, float *sqrDist)
{
    Mesh& mesh = m_document->mesh();

    Mesh::Halfedge h = mesh.firstHalfedge(curve);
    Mesh::Halfedge closest;
    float closestSqrDist = std::numeric_limits<float>::infinity();
    while(mesh.isValid(h))
    {
        float dist = m_document->edgeSqrDist(mesh.edge(h), p);
        if(dist < closestSqrDist)
        {
            closest = h;
            closestSqrDist = dist;
        }
        h = mesh.nextCurveHalfedge(h);
    }

    Vector p0 = mesh.position(mesh.fromVertex(closest));
    Vector p1 = mesh.position(mesh.toVertex(closest));
    Vector v = p - p0;
    Vector e =  p1 - p0;
    float alpha = std::min(std::max(v.dot(e) / e.squaredNorm(), 0.f), 1.f);
    float pos0 = mesh.fromCurvePos(closest);
    float pos1 = mesh.toCurvePos(closest);
    float pos = (1 - alpha) * pos0 + alpha * pos1;

    if(side)
    {
        *side = ((Eigen::Matrix2f() << e, v).finished().determinant() > 0)?
                    Mesh::LEFT: Mesh::RIGHT;
    }

    if(sqrDist)
    {
        *sqrDist = closestSqrDist;
    }

    return pos;
}


void Editor::drawCurve(Mesh::Curve curve, float width, const Eigen::Vector4f color)
{
    Mesh& mesh = m_document->mesh();

    Mesh::Halfedge h = mesh.firstHalfedge(curve);
    Eigen::Vector3f p; p << mesh.position(mesh.fromVertex(h)), 0;

    m_lineRenderer.addPoint(p, width, color);
    while(mesh.isValid(h))
    {
        p << mesh.position(mesh.toVertex(h)), 0;
        m_lineRenderer.addPoint(p, width, color);
        h = mesh.nextCurveHalfedge(h);
    }
    m_lineRenderer.endLine();
}


void Editor::drawGradientStops(float innerRadius, float outerRadius)
{
    for(GradientStopList::const_iterator gsit = m_gradientStops.begin();
        gsit != m_gradientStops.end(); ++gsit)
    {
        Eigen::Vector3f p;
        p << gsit->position, 0;
        Mesh::NodeValue outerColor = (gsit->color.head<3>().sum() < 1.5)?
                    Mesh::NodeValue(1, 1, 1, 1):
                    Mesh::NodeValue(0, 0, 0, 1);
        m_pointRenderer.addPoint(p, outerRadius, outerColor);
        m_pointRenderer.addPoint(p, innerRadius, gsit->color);
    }

    if(m_dummyStop.curve.isValid())
    {
        Eigen::Vector3f p;
        p << m_dummyStop.position, 0;
        Mesh::NodeValue outerColor = (m_dummyStop.color.head<3>().sum() < 1.5)?
                    Mesh::NodeValue(1, 1, 1, 1):
                    Mesh::NodeValue(0, 0, 0, 1);
        m_pointRenderer.addPoint(p, outerRadius, outerColor);
        m_pointRenderer.addPoint(p, innerRadius, m_dummyStop.color);
    }
}


Mesh& Editor::mesh()
{
    return m_document->solvedMesh();
}


const Mesh& Editor::mesh() const
{
    return m_document->solvedMesh();
}
