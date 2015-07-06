/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <GL/glew.h>

#include "shaders.h"

#include "glPointRenderer.h"


GLPointRenderer::GLPointRenderer()
    : m_defaultRadius(1), m_defaultColor(0, 0, 0, 1),
      m_vao(0), m_buffer(0)
{
}


float GLPointRenderer::defaultRadius() const
{
    return m_defaultRadius;
}


void GLPointRenderer::setDefaultRadius(float defaultRadius)
{
    m_defaultRadius = defaultRadius;
}


const Eigen::Vector4f& GLPointRenderer::defaultColor() const
{
    return m_defaultColor;
}


void GLPointRenderer::setDefaultColor(const Eigen::Vector4f& defaultColor)
{
    m_defaultColor = defaultColor;
}


void GLPointRenderer::clear()
{
    m_points.clear();
}


void GLPointRenderer::addPoint(const Eigen::Vector3f& pos, float radius,
              const Eigen::Vector4f& color)
{
    Point point;
    point.pos = pos;
    point.radius = radius >= 0? radius: m_defaultRadius;
    point.color = color != Eigen::Vector4f::Constant(-1)? color: m_defaultColor;

    m_points.push_back(point);
}


void GLPointRenderer::upload()
{
    PATATE_ASSERT_NO_GL_ERROR();

	if(m_points.empty()) {
		return;
	}

    if(m_shader.status() == PatateCommon::Shader::UNINITIALIZED)
    {
        m_shader.create();

        bool bRes = true;

        bRes &= m_shader.addShader(GL_VERTEX_SHADER,
                                   vert_points_glsl);
        bRes &= m_shader.addShader(GL_GEOMETRY_SHADER,
                                   geom_points_glsl);
        bRes &= m_shader.addShader(GL_FRAGMENT_SHADER,
                                   frag_points_glsl);

        bRes &= m_shader.finalize();
        assert(bRes);

        m_positionLoc = glGetAttribLocation(m_shader.getShaderId(), "vx_position");
        m_colorLoc = glGetAttribLocation(m_shader.getShaderId(), "vx_color");
        m_viewMatrixLoc = m_shader.getUniformLocation("viewMatrix");
        m_viewportSizeLoc = m_shader.getUniformLocation("viewportSize");
    }

    if(!m_vao)
        glGenVertexArrays(1, &m_vao);

    if(!m_buffer)
        glGenBuffers(1, &m_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, m_buffer);
    glBufferData(GL_ARRAY_BUFFER, m_points.size() * sizeof(Point),
                 &m_points[0], GL_DYNAMIC_DRAW);

    PATATE_ASSERT_NO_GL_ERROR();
}


void GLPointRenderer::render(const Eigen::Matrix4f& viewMatrix,
                             const Eigen::Vector2f& viewportSize)
{
    PATATE_ASSERT_NO_GL_ERROR();

    if(m_points.empty())
        return;

    m_shader.use();

    glEnable(GL_PROGRAM_POINT_SIZE);

    glBindVertexArray(m_vao);

    glBindBuffer(GL_ARRAY_BUFFER, m_buffer);
    if(m_positionLoc >= 0)
    {
        glEnableVertexAttribArray(m_positionLoc);
        glVertexAttribPointer(m_positionLoc, 4, GL_FLOAT, false,
                              sizeof(Point), 0);
    }
    if(m_colorLoc >= 0)
    {
        glEnableVertexAttribArray(m_colorLoc);
        glVertexAttribPointer(m_colorLoc, 4, GL_FLOAT, false,
                              sizeof(Point), (void*)(4 * sizeof(float)));
    }

    if(m_viewMatrixLoc >= 0)
        glUniformMatrix4fv(m_viewMatrixLoc, 1, false, viewMatrix.data());
    if(m_viewportSizeLoc >= 0)
        glUniform2fv(m_viewportSizeLoc, 1, viewportSize.data());

    glDrawArrays(GL_POINTS, 0, m_points.size());

    if(m_positionLoc >= 0)
        glDisableVertexAttribArray(m_positionLoc);
    if(m_colorLoc >= 0)
        glDisableVertexAttribArray(m_colorLoc);

    PATATE_ASSERT_NO_GL_ERROR();
}
