#ifndef GL_LINE_RENDERER
#define GL_LINE_RENDERER


#include <vector>

#include <Eigen/Dense>

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

private:
    float m_defaultWidth;
    Eigen::Vector4f m_defaultColor;

    bool m_startNewLine;
    std::vector<Point> m_points;
    std::vector<int> m_firsts;
    std::vector<int> m_sizes;

    Patate::Shader m_shader;
    unsigned m_vao;
    unsigned m_buffer;

    int m_positionLoc;
    int m_colorLoc;
    int m_viewMatrixLoc;
    int m_viewportSizeLoc;
};


#endif
