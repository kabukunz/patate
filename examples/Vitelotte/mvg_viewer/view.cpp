/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


#include <GL/glew.h>

#include <QMouseEvent>
#include <QWheelEvent>

#include "view.h"


View::View(QWidget* parent)
    : QGLWidget(parent),
      m_initialized(false),
      m_wireMode(WIRE_NONE) {
}


bool View::canRotate() const {
    return m_mesh.nDims() != 2;
}


void View::openMvg(QString filename) {
    if(!m_initialized) {
        // Defer loading to initializeGL()
        m_meshFilename = filename;
        return;
    }

    std::ifstream in(filename.toLocal8Bit().data());
    if(in.good()) {
        Vitelotte::MVGWithCurvesReader<Mesh> reader;
        if(reader.read(in, m_mesh)) {
            m_renderer.updateBuffers(m_mesh);
            resetTrackball();  // Call update
        }
    } else {
        std::cerr << "Can not open file \"" << filename.toLocal8Bit().data() << "\".\n";
    }
}


void View::resetTrackball() {
    if(!m_mesh.nVertices()) {
        return;
    }

    Eigen::AlignedBox3f bb;
    for(Mesh::VertexIterator vit = m_mesh.verticesBegin();
        vit != m_mesh.verticesEnd(); ++vit) {
        bb.extend(position(*vit));
    }

    m_trackball.setSceneCenter(bb.center());
    m_trackball.setSceneRadius(bb.sizes().maxCoeff());
    m_trackball.setSceneDistance(m_trackball.sceneRadius() * 3.f);
    m_trackball.setNearFarOffsets(-m_trackball.sceneRadius() * 100.f,
                                   m_trackball.sceneRadius() * 100.f);
    m_trackball.setSceneOrientation(Eigen::Quaternionf::Identity());
    m_trackball.setScreenViewport(Eigen::AlignedBox2f(
                                    Eigen::Vector2f(0, 0),
                                    Eigen::Vector2f(width(), height())));

    update();
}


void View::setWireMode(WireMode wm) {
    m_wireMode = wm;
    update();
}


void View::initializeGL() {
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
    else if(!GLEW_VERSION_4_0)
    {
        std::cerr << "OpenGL 4.0 not supported. Aborting.\n";
        abort();
    }

    glGetError();  // FIXME: avoid a GL error, but why glewInit fail ?

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glClearColor(.5, .5, .5, 1.);

    // Must be set before calling openMvg.
    m_initialized = true;
    if(!m_meshFilename.isNull()) {
        openMvg(m_meshFilename);
    }
}


void View::resizeGL(int width, int height) {
    glViewport(0, 0, width, height);
    m_trackball.setScreenViewport(Eigen::AlignedBox2f(
                                    Eigen::Vector2f(0, 0),
                                    Eigen::Vector2f(width, height)));
}


void View::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if(m_mesh.nVertices()) {
        Eigen::Matrix4f projMatrix = m_trackball.computeProjectionMatrix();
        Eigen::Matrix4f viewMatrix = m_trackball.computeViewMatrix();
        Eigen::Matrix4f trans = projMatrix * viewMatrix;

        if(m_wireMode < WIRE_ONLY_SIMPLE) {
            glDisable(GL_FRAMEBUFFER_SRGB);
            m_renderer.render(trans);
        }

        Eigen::Vector2f viewportSize(width(), height());

        glEnable(GL_FRAMEBUFFER_SRGB);
        switch(m_wireMode) {
        case WIRE_SIMPLE:
        case WIRE_ONLY_SIMPLE: {
            m_renderer.renderWireframe(trans, viewportSize, .5);
            break;
        }
        case WIRE_NODES:
        case WIRE_ONLY_NODES: {
            // Node renderer require some attributes.
            if((m_mesh.getAttributes() & Mesh::QUADRATIC_FLAGS) == Mesh::QUADRATIC_FLAGS) {
                m_nodeRenderer.update(m_mesh, trans, viewportSize);
                m_nodeRenderer.render();
            }
            break;
        }
        default:
            break;
        }
    }
}


void View::mousePressEvent(QMouseEvent* event) {
    Eigen::Vector2f pos(event->x(), event->y());
    if(event->button() == Qt::LeftButton) {
        m_trackball.startTranslation(pos);
        update();
        event->accept();
    } else if(canRotate() && event->button() == Qt::RightButton) {
        m_trackball.startRotation(pos);
        update();
        event->accept();
    }
}


void View::mouseReleaseEvent(QMouseEvent* event) {
    if(event->button() == Qt::LeftButton) {
        m_trackball.endTranslation();
        update();
        event->accept();
    } else if(canRotate() && event->button() == Qt::RightButton) {
        m_trackball.endRotation();
        update();
        event->accept();
    }
}


void View::mouseMoveEvent(QMouseEvent* event) {
    Eigen::Vector2f pos(event->x(), event->y());
    if(m_trackball.state() == Trackball::Translating) {
        m_trackball.dragTranslate(pos);
        update();
        event->accept();
    } else if(canRotate() && m_trackball.state() == Trackball::Rotating) {
        m_trackball.dragRotate(pos);
        update();
        event->accept();
    }
}


void View::wheelEvent(QWheelEvent* event) {
    float factor = (event->delta() > 0)? 1.1f: 1.f/1.1f;
    if(event->modifiers() == 0) {
        m_trackball.zoom(factor);
    } else if(event->modifiers() == Qt::ControlModifier) {
        m_trackball.dollyZoom(factor);
    } else if(event->modifiers() == Qt::ShiftModifier) {
        m_trackball.grow(1.f / factor);
    }
    update();
    event->accept();
}


Eigen::Vector3f View::position(Mesh::Vertex vx) const {
    Mesh::Vector v = m_mesh.position(vx);
    return (v.rows() == 2)?
        (Eigen::Vector3f() << v, 0).finished():
        v.head<3>();
}
