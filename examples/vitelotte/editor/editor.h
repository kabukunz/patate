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
#include "../common/glPointRenderer.h"
#include "../common/glLineRenderer.h"
#include "../common/vgNodeRenderer.h"

#include "document.h"


class QMouseEvent;
class QWheelEvent;

enum EditMode
{
    EDIT_NODES  = 0,
    EDIT_CURVES = 1
};


class Editor : public QGLWidget
{
    Q_OBJECT

public:
    typedef Vitelotte::VGMeshRenderer<Mesh> Renderer;

    typedef Mesh::Vector Vector;
    typedef Mesh::NodeValue NodeValue;

    typedef Eigen::AlignedBox<Scalar, 2> Box;

public:
    explicit Editor(QWidget* parent=0);
    virtual ~Editor();

    bool showConstraints() const { return m_showConstraints; }
    bool showWireframe() const { return m_showWireframe; }

    Eigen::Vector2f sceneFromView(const QPointF& view) const;

    Eigen::Vector2f screenToNormalized(const QPointF& screen) const;
    QPointF normalizedToScreen(const Eigen::Vector2f& normalized) const;

public slots:
    void centerView();
    void setDocument(Document* document);
    void updateMesh();
    void updateBuffers();
    void updateRenderers();
    void setShowConstraints(bool enable);
    void setShowWireframe(bool enable);
    void setShowMesh(int type);
    void setEditMode(int mode);
    void setPaintColor(const QColor& color);

public:
    virtual void initializeGL();
    virtual void resizeGL(int width, int height);
    virtual void paintGL();

    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);

    virtual void wheelEvent(QWheelEvent* event);


protected:
    struct GradientStop
    {
        Eigen::Vector2f position;
        Eigen::Vector4f color;

        Mesh::Curve curve;
        unsigned which;
        float gpos;

        inline GradientStop()
            : position(), color(), curve(), which(), gpos()
        {}
        inline GradientStop(Mesh::Curve curve, unsigned which, float gpos,
                     const Eigen::Vector2f& position, const Eigen::Vector4f& color)
            : position(position), color(color), curve(curve), which(which), gpos(gpos)
        {}

        inline bool operator==(const GradientStop& other) const
        {
            return position == other.position && color == other.color &&
                    curve == other.curve && which == other.which && gpos == other.gpos;
        }
        inline bool operator!=(const GradientStop& other) const
        { return !(*this == other); }
    };

    typedef std::vector<GradientStop> GradientStopList;

    enum InputState
    {
        STATE_IDLE,
        STATE_PAN_VIEW,
        STATE_GRABED_GRADIENT_STOP,
        STATE_DRAG_GRADIENT_STOP
    };

protected:
    float zoom() const;

    void beginPanView(const Vector& scenePos);
    void endPanView();
    void panView(const Vector& scenePos);

    void pickVertexOrEdge(const Vector& scenePos);
    void pickConstraint(const Vector& scenePos);

    GradientStop* pickGradientStop(const Vector& scenePos);
    void beginDragStop(GradientStop* gs);
    void endDragStop();
    void dragStop(const Vector& scenePos);

    void makeNewStop(GradientStop& gs, const Vector& scenePos);
    void showDummyStop(const Vector& scenePos);
    void hideDummyStop();

    void setStopColor(const GradientStop& gs);
    void addGradientStop(const GradientStop& gs);
    void removeGradientStop(const GradientStop& gs);

    bool trySetPointConstraint(const Vector& pos);

    void doUpdateRenderers();

    Mesh::Vector computeStopPosition(Mesh::Curve curve, float pos, float offset,
                                     Mesh::Halfedge* from = 0);
    void addGradientStops(Mesh::Curve curve, unsigned which, float offset);
    GradientStop* closestGradientStop(const Vector& p, float* dist);
    float closestPos(Mesh::Curve curve, const Vector& p,
                     unsigned* which=0, float* sqrDist=0);

    void drawCurve(Mesh::Curve curve, float width, const Eigen::Vector4f color);
    void drawGradientStops(float innerRadius, float outerRadius);

protected:
    Mesh& mesh();
    const Mesh& mesh() const;

protected:
    Document* m_document;

    bool m_initialized;
    bool m_showConstraints;
    bool m_showWireframe;
    Document::MeshType m_nodeMeshType;
    EditMode m_editMode;

    OrthographicCamera m_camera;
    InputState m_inputState;
    Eigen::Vector2f m_dragPos;
    GradientStop m_dragGradientStop;

    Renderer m_renderer;

    Mesh::NodeValue m_paintColor;
    GradientStopList m_gradientStops;
    GradientStop m_dummyStop;

    bool m_rendererDirty;
    GLPointRenderer m_pointRenderer;
    GLLineRenderer m_lineRenderer;
    VGNodeRenderer m_nodeRenderer;
};


#endif
