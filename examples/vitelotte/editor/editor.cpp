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
      m_showWireframe(true),
      m_nodeMeshType(Document::BASE_MESH),
      m_editMode(EDIT_NODES),
      m_camera(),
      m_drag(false),
      m_dragPos()
{
}


Editor::~Editor()
{
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
        connect(m_document, SIGNAL(selectionChanged()),
                this, SLOT(updateSelection()));
    }
}


void Editor::updateBuffers()
{
    assert(m_document);
    m_renderer.setMesh(&mesh());
    update();
}


void Editor::updateSelection()
{
    assert(m_document);

    Mesh& mesh = m_document->mesh();
    MeshSelection sel = m_document->selection();
    m_pointRenderer.clear();
    m_lineRenderer.clear();
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

        Mesh::Curve c = sel.curve();
        Mesh::Halfedge h;
        Eigen::Vector3f p;

        h = mesh.firstHalfedge(c);
        p << mesh.position(mesh.fromVertex(h)), 0;
        m_lineRenderer.addPoint(p, outerWidth, outerColor);
        while(mesh.isValid(h))
        {
            p << mesh.position(mesh.toVertex(h)), 0;
            m_lineRenderer.addPoint(p, outerWidth, outerColor);
            h = mesh.nextCurveHalfedge(h);
        }
        m_lineRenderer.endLine();

        h = mesh.firstHalfedge(c);
        p << mesh.position(mesh.fromVertex(h)), 0;
        m_lineRenderer.addPoint(p, innerWidth, innerColor);
        while(mesh.isValid(h))
        {
            p << mesh.position(mesh.toVertex(h)), 0;
            m_lineRenderer.addPoint(p, innerWidth, innerColor);
            h = mesh.nextCurveHalfedge(h);
        }
        m_lineRenderer.endLine();
    }
    m_pointRenderer.upload();
    m_lineRenderer.upload();

    update();
}


void Editor::setShowWireframe(bool enable)
{
    if(enable != m_showWireframe)
    {
        m_showWireframe = enable;
        update();
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

    glEnable(GL_FRAMEBUFFER_SRGB);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glClearColor(.5, .5, .5, 1.);

    m_initialized = true;
    m_renderer.initialize();
    if(m_document) {
        m_renderer.setMesh(&mesh());
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
        m_defaultShader.viewMatrix() = m_camera.projectionMatrix();
        m_renderer.render(m_defaultShader);

        m_wireframeShader.viewMatrix() = m_camera.projectionMatrix();
        m_wireframeShader.setLineWidth(.5);
        m_wireframeShader.setWireframeColor(Eigen::Vector4f(.5, .5, .5, 1.));
        m_wireframeShader.setZoom(width() / m_camera.getViewBox().sizes()(0));
        //m_renderer.render(m_wireframeShader);

        Eigen::Vector2f viewportSize(width(), height());
        m_lineRenderer.render(m_wireframeShader.viewMatrix(), viewportSize);
        m_pointRenderer.render(m_wireframeShader.viewMatrix(), viewportSize);

        if(m_showWireframe)
        {
            m_nodeRenderer.update(m_document->getMesh(m_nodeMeshType),
                                  width() / m_camera.getViewBox().sizes()(0));
            m_nodeRenderer.render(m_wireframeShader.viewMatrix(), viewportSize);
        }
    }
}


void Editor::mousePressEvent(QMouseEvent* event)
{
    if(!m_drag && event->button() == Qt::LeftButton)
    {
        Eigen::Vector2f norm = screenToNormalized(event->localPos());
        Eigen::Vector2f cam = m_camera.normalizedToCamera(norm);

        float selDist = 8 * m_camera.getViewBox().sizes()(0) / width();
        MeshSelection sel;

        if(m_editMode == EDIT_NODES)
        {
            float vSqrDist;
            Mesh::Vertex vSel = m_document->closestVertex(cam, &vSqrDist);

            float eSqrDist;
            Mesh::Edge eSel = m_document->closestEdge(cam, &eSqrDist);

            if(vSqrDist < selDist*selDist)
                sel = MeshSelection(vSel);
            else if(eSqrDist < selDist*selDist)
                sel = MeshSelection(eSel);
        }
        else {
            float pcSqrDist;
            Mesh::PointConstraint pcSel =
                    m_document->closestPointConstraint(cam, &pcSqrDist);

            float cSqrDist;
            Mesh::Curve cSel = m_document->closestCurve(cam, &cSqrDist);

            if(pcSqrDist < selDist*selDist)
                sel = MeshSelection(pcSel);
            else if(cSqrDist < selDist*selDist)
                sel = MeshSelection(cSel);
        }

        m_document->setSelection(sel);
    }
    if(!m_drag && event->button() == Qt::MidButton)
    {
        grabMouse();
        m_drag = true;
        m_dragPos = screenToNormalized(event->localPos());
    }
}


void Editor::mouseReleaseEvent(QMouseEvent* event)
{
    if(m_drag && event->button() == Qt::MidButton)
    {
        releaseMouse();
        m_drag = false;
    }
}

void Editor::mouseMoveEvent(QMouseEvent* event)
{
    if(m_drag)
    {
        Eigen::Vector2f norm = screenToNormalized(event->localPos());
        m_camera.normalizedTranslate(norm - m_dragPos);
        m_dragPos = norm;
        update();
    }
}


void Editor::wheelEvent(QWheelEvent* event)
{
    m_camera.normalizedZoom(
                screenToNormalized(event->posF()),
                event->delta() > 0? 1.1: 1/1.1);
    update();
}


Mesh& Editor::mesh()
{
    return m_document->solvedMesh();
}


const Mesh& Editor::mesh() const
{
    return m_document->solvedMesh();
}
