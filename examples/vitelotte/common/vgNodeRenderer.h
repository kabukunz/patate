#ifndef _PATATE_EXAMPLES_COMMON_VG_NODE_RENDERER_H
#define _PATATE_EXAMPLES_COMMON_VG_NODE_RENDERER_H


#include <Patate/vitelotte.h>

#include "glPointRenderer.h"
#include "glLineRenderer.h"


class VGNodeRenderer
{
public:
    typedef Vitelotte::VGMesh<float> Mesh;

public:
    VGNodeRenderer();

    void clear();
    void update(const Mesh& mesh, float zoom);

    void render(const Eigen::Matrix4f& transform,
                const Eigen::Vector2f& viewportSize);

private:
    float m_nodeOffset;

    GLPointRenderer m_pointRenderer;
    GLLineRenderer m_lineRenderer;
};


#endif
