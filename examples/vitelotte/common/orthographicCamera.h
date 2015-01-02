#ifndef _ORTHOGRAPHIC_CAMERA_H_
#define _ORTHOGRAPHIC_CAMERA_H_


#include <Eigen/Geometry>


class OrthographicCamera
{
public:
    typedef Eigen::AlignedBox3f ViewBox;

public:
    OrthographicCamera();

    const ViewBox& getViewBox() const;
    void setViewBox(const ViewBox& viewBox);

    Eigen::Vector2f normalizedToCamera(const Eigen::Vector2f& norm) const;
    Eigen::Vector2f cameraToNormalized(const Eigen::Vector2f& camera) const;

    void changeAspectRatio(float ratio);
    void translate(const Eigen::Vector2f& offset);
    void zoom(const Eigen::Vector2f& center, float zoom);

    void normalizedTranslate(const Eigen::Vector2f& offset);
    void normalizedZoom(const Eigen::Vector2f& center, float zoom);

    Eigen::Matrix4f projectionMatrix() const;

private:
    ViewBox m_viewBox;
};


#endif
