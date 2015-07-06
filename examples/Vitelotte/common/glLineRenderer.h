/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _EXAMPLES_VITELOTTE_COMMON_GL_LINE_RENDERER_
#define _EXAMPLES_VITELOTTE_COMMON_GL_LINE_RENDERER_


#include <vector>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "Patate/common/gl_utils/shader.h"


class GLLineRenderer
{
public:
    struct Point
    {
        Eigen::Vector3f pos;
        float width;
        Eigen::Vector4f color;
    };

public:
    GLLineRenderer();

    float defaultWidth() const;
    void setDefaultWidth(float defaultWidth);

    const Eigen::Vector4f& defaultColor() const;
    void setDefaultColor(const Eigen::Vector4f& defaultColor);

    void clear();
    void addPoint(const Eigen::Vector3f& pos, float width=-1,
                  const Eigen::Vector4f& color=Eigen::Vector4f::Constant(-1));
    void endLine();

    void upload();
    void render(const Eigen::Matrix4f& viewMatrix,
                const Eigen::Vector2f& viewportSize);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    typedef std::vector<Point, Eigen::aligned_allocator<Point> > PointVector;

private:
    float m_defaultWidth;
    Eigen::Vector4f m_defaultColor;

    bool m_startNewLine;
    PointVector m_points;
    std::vector<int> m_firsts;
    std::vector<int> m_sizes;

    PatateCommon::Shader m_shader;
    unsigned m_vao;
    unsigned m_buffer;

    int m_positionLoc;
    int m_colorLoc;
    int m_viewMatrixLoc;
    int m_viewportSizeLoc;
};


#endif
