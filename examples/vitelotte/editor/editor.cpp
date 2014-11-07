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
      m_drag(false)
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
    scale.head<2>() = bb.sizes().array() / Eigen::Array2f(width(), height()) * .6;
    if(scale(0) > scale(1))
        scale *= float(width());
    else
        scale *= float(height());
    scale(2) = 1;
    m_camera.setViewBox(OrthographicCamera::ViewBox(
        center - scale,
        center + scale));
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

        connect(m_document, SIGNAL(meshUpdated()), this, SLOT(updateBuffers()));
    }
}


void Editor::updateBuffers()
{
    assert(m_document);
    m_renderer.setMesh(&mesh());
}


void Editor::initializeGL()
{
    std::cout << "OpenGL Vendor:       " << glGetString(GL_VENDOR) << "\n";
    std::cout << "OpenGL Renderer:     " << glGetString(GL_RENDERER) << "\n";
    std::cout << "OpenGL Version:      " << glGetString(GL_VERSION) << "\n";
    std::cout << "OpenGL GLSL Version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << "\n";

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
    //    if (m_info.flags.debug)
    //    {
    //        if (GLEW_VERSION_4_3)
    //        {
    //            glDebugMessageCallback(debugCallback, this);
    //            glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    //        }
    //        else if (glfwExtensionSupported("GL_ARB_debug_output"))
    //        {
    //            glDebugMessageCallbackARB(debugCallback, this);
    //            glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS_ARB);
    //        }
    //    }

    glEnable(GL_FRAMEBUFFER_SRGB);

    glClearColor(.5, .5, .5, 1.);

    m_defaultShaders.showWireframe() = true;
    m_defaultShaders.lineWidth() = .5;
    m_defaultShaders.wireframeColor() = Eigen::Vector4f(.5, .5, .5, 1.);

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

    m_defaultShaders.viewMatrix() = m_camera.projectionMatrix();
    m_defaultShaders.zoom() = width() / m_camera.getViewBox().sizes()(0);

    if(m_document) {
        m_renderer.render(m_defaultShaders);
    }
}


void Editor::mousePressEvent(QMouseEvent* event)
{
    if(!m_drag && event->button() == Qt::LeftButton)
    {
        Eigen::Vector2f norm = screenToNormalized(event->localPos());
        Eigen::Vector2f cam = m_camera.normalizedToCamera(norm);

        Mesh::Edge sel = m_document->closestEdge(cam);
        m_document->setSelectedEdge(sel);

//        Mesh& m = m_document->mesh();
//        Mesh::Halfedge h0 = m.halfedge(sel, 0);
//        Mesh::Halfedge h1 = m.halfedge(sel, 1);

//        Mesh::Node* nodes[6] =
//        {
//            &m.vertexValueNode(h0), &m.vertexFromValueNode(h0), &m.edgeValueNode(h0),
//            &m.vertexValueNode(h1), &m.vertexFromValueNode(h1), &m.edgeValueNode(h1)
//        };
//        for(int i=0; i<6; ++i)
//        {
//            if(!nodes[i]->isValid())
//                *nodes[i] = m.addNode();
//            m.nodeValue(*nodes[i]) = (Eigen::Vector4f() <<
//                Eigen::Vector3f::Random() / 2 + Eigen::Vector3f::Constant(.5),
//                1).finished();
//        }

//        m_document->solve();
//        updateBuffers();
//        update();
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
        updateGL();
    }
}


void Editor::wheelEvent(QWheelEvent* event)
{
    m_camera.normalizedZoom(
                screenToNormalized(event->posF()),
                event->delta() > 0? 1.1: 1/1.1);
    updateGL();
}


Editor::Mesh& Editor::mesh()
{
    return m_document->mesh();
}


const Editor::Mesh& Editor::mesh() const
{
    return m_document->mesh();
}
