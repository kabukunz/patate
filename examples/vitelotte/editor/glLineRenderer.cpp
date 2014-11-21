#include <GL/glew.h>

#include "glLineRenderer.h"


extern const char* vert_lines_glsl;
extern const char* geom_lines_glsl;
extern const char* frag_lines_glsl;


GLLineRenderer::GLLineRenderer()
    : m_defaultWidth(1), m_defaultColor(0, 0, 0, 1),
      m_startNewLine(true), m_vao(0), m_buffer(0)
{
}


float GLLineRenderer::defaultWidth() const
{
    return m_defaultWidth;
}


void GLLineRenderer::setDefaultWidth(float defaultWidth)
{
    m_defaultWidth = defaultWidth;
}


const Eigen::Vector4f& GLLineRenderer::defaultColor() const
{
    return m_defaultColor;
}


void GLLineRenderer::setDefaultColor(const Eigen::Vector4f& defaultColor)
{
    m_defaultColor = defaultColor;
}


void GLLineRenderer::clear()
{
    m_startNewLine = true;
    m_points.clear();
    m_firsts.clear();
    m_sizes.clear();
}


void GLLineRenderer::addPoint(const Eigen::Vector3f& pos, float width,
              const Eigen::Vector4f& color)
{
    if(m_startNewLine)
    {
        m_startNewLine = false;
        m_firsts.push_back(m_points.size());
        m_sizes.push_back(0);
    }

    Point point;
    point.pos = pos;
    point.width = width >= 0? width: m_defaultWidth;
    point.color = color != Eigen::Vector4f::Constant(-1)? color: m_defaultColor;

    m_points.push_back(point);
    ++m_sizes.back();
}


void GLLineRenderer::endLine()
{
    m_startNewLine = true;
}


void GLLineRenderer::upload()
{
    PATATE_ASSERT_NO_GL_ERROR();

    if(m_shader.status() == Patate::Shader::Uninitialized)
    {
        m_shader.create();

        bool bRes = true;

        bRes &= m_shader.addShader(GL_VERTEX_SHADER,
                                   vert_lines_glsl);
        bRes &= m_shader.addShader(GL_GEOMETRY_SHADER,
                                   geom_lines_glsl);
        bRes &= m_shader.addShader(GL_FRAGMENT_SHADER,
                                   frag_lines_glsl);

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


void GLLineRenderer::render(const Eigen::Matrix4f& viewMatrix,
                            const Eigen::Vector2f& viewportSize)
{
    PATATE_ASSERT_NO_GL_ERROR();

    if(m_sizes.empty())
        return;

    m_shader.use();

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

    glMultiDrawArrays(GL_LINE_STRIP, &m_firsts[0], &m_sizes[0], m_sizes.size());

    if(m_positionLoc >= 0)
        glDisableVertexAttribArray(m_positionLoc);
    if(m_colorLoc >= 0)
        glDisableVertexAttribArray(m_colorLoc);

    PATATE_ASSERT_NO_GL_ERROR();
}
