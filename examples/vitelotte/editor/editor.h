#ifndef _EDITOR_H_
#define _EDITOR_H_


//#include <cstdlib>
//#include <cstdio>

#include <Eigen/Dense>

#include <GL/glew.h>

#include <QGLWidget>

#include "Patate/vitelotte.h"
#include "Patate/vitelotte_gl.h"

#include "../common/orthographicCamera.h"
#include "glPointRenderer.h"
#include "glLineRenderer.h"


class QMouseEvent;
class QWheelEvent;

class Document;


class Editor : public QGLWidget
{
    Q_OBJECT

public:
    typedef float Scalar;

    typedef Vitelotte::VGMesh<Scalar, 2, 4> Mesh;
    typedef Vitelotte::VGMeshRenderer<Mesh> Renderer;

    typedef Mesh::Vector Vector;
    typedef Mesh::NodeValue NodeValue;

    typedef Eigen::AlignedBox<Scalar, 2> Box;


public:
    explicit Editor(QWidget* parent=0);
    virtual ~Editor();

    Eigen::Vector2f screenToNormalized(const QPointF& screen) const;
    QPointF normalizedToScreen(const Eigen::Vector2f& normalized) const;

public slots:
    void centerView();
    void setDocument(Document* document);
    void updateBuffers();
    void updateSelection();

public:
    virtual void initializeGL();
    virtual void resizeGL(int width, int height);
    virtual void paintGL();

    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);

    virtual void wheelEvent(QWheelEvent* event);


private:
    Mesh& mesh();
    const Mesh& mesh() const;

private:
    Document* m_document;

    bool m_initialized;

    OrthographicCamera m_camera;
    bool m_drag;
    Eigen::Vector2f m_dragPos;

    Renderer m_renderer;
    Vitelotte::VGMeshRendererDefaultShader m_defaultShader;
    Vitelotte::VGMeshRendererWireframeShader m_wireframeShader;

    GLPointRenderer m_pointRenderer;
    GLLineRenderer m_lineRenderer;

};


#endif
