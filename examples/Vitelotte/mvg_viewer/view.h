/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/


#ifndef _MVG_VIEWER_VIEW_
#define _MVG_VIEWER_VIEW_


#include <GL/glew.h>

#include <QGLWidget>

#include <Patate/vitelotte.h>
#include <Patate/vitelotte_gl.h>

#include "../common/trackball.h"
#include "../common/vgNodeRenderer.h"


class View : public QGLWidget {
    Q_OBJECT

    typedef float Scalar;
    typedef Vitelotte::DCMesh<Scalar, Vitelotte::Dynamic, Vitelotte::Dynamic> Mesh;
    typedef Vitelotte::VGMeshRenderer<Mesh> Renderer;

public:
    enum WireMode {
        WIRE_NONE,
        WIRE_SIMPLE,
        WIRE_NODES,
        WIRE_ONLY_SIMPLE,
        WIRE_ONLY_NODES
    };

public:
    View(QWidget* parent = NULL);

    bool canRotate() const;

public slots:
    void openMvg(QString filename);
    void resetTrackball();
    void setWireMode(WireMode wm);

protected:
    virtual void initializeGL();
    virtual void resizeGL(int width, int height);
    virtual void paintGL();

    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);

    virtual void wheelEvent(QWheelEvent* event);

    Eigen::Vector3f position(Mesh::Vertex vx) const;

private:
    QString             m_meshFilename;
    Mesh                m_mesh;

    Renderer            m_renderer;
    VGNodeRenderer      m_nodeRenderer;

    Trackball           m_trackball;

    bool                m_initialized;
    WireMode            m_wireMode;
};


#endif
