#ifndef GL_POINT_RENDERER
#define GL_POINT_RENDERER


#include <vector>

#include <Eigen/Dense>

#include "Patate/common/gl_utils/shader.h"


class GLPointRenderer
{
public:
    struct Point
    {
        Eigen::Vector3f pos;
        float radius;
        Eigen::Vector4f color;
    };

public:
    GLPointRenderer();

    float defaultRadius() const;
    void setDefaultRadius(float defaultRadius);

    const Eigen::Vector4f& defaultColor() const;
    void setDefaultColor(const Eigen::Vector4f& defaultColor);

    void clear();
    void addPoint(const Eigen::Vector3f& pos, float radius=-1,
                  const Eigen::Vector4f& color=Eigen::Vector4f::Constant(-1));

    void upload();
    void render(const Eigen::Matrix4f& viewMatrix,
                const Eigen::Vector2f& viewportSize);

private:
    float m_defaultRadius;
    Eigen::Vector4f m_defaultColor;

    std::vector<Point> m_points;

    Patate::Shader m_shader;
    unsigned m_vao;
    unsigned m_buffer;

    int m_positionLoc;
    int m_colorLoc;
    int m_viewMatrixLoc;
    int m_viewportSizeLoc;
};


#endif
