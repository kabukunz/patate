#include "orthographicCamera.h"


OrthographicCamera::OrthographicCamera()
    : m_viewBox(Eigen::Vector3f::Constant(-1), Eigen::Vector3f::Constant(1))
{
}


const OrthographicCamera::ViewBox& OrthographicCamera::getViewBox() const
{
    return m_viewBox;
}


void OrthographicCamera::setViewBox(const ViewBox& viewBox)
{
    m_viewBox = viewBox;
}


Eigen::Vector2f
OrthographicCamera::normalizedToCamera(const Eigen::Vector2f& norm) const
{
    float rpl = m_viewBox.max()(0) + m_viewBox.min()(0);
    float tpb = m_viewBox.max()(1) + m_viewBox.min()(1);

    float rml = m_viewBox.max()(0) - m_viewBox.min()(0);
    float tmb = m_viewBox.max()(1) - m_viewBox.min()(1);

    return Eigen::Vector2f(
                (norm.x() * rml + rpl) / 2.f,
                (norm.y() * tmb + tpb) / 2.f);
}


Eigen::Vector2f
OrthographicCamera::cameraToNormalized(const Eigen::Vector2f& camera) const
{
    float rpl = m_viewBox.max()(0) + m_viewBox.min()(0);
    float tpb = m_viewBox.max()(1) + m_viewBox.min()(1);

    float rml = m_viewBox.max()(0) - m_viewBox.min()(0);
    float tmb = m_viewBox.max()(1) - m_viewBox.min()(1);

    return Eigen::Vector2f(
                camera.x() * (2.f/rml) - rpl/rml,
                camera.y() * (2.f/tmb) - tpb/tmb);
}


void OrthographicCamera::changeAspectRatio(float ratio)
{
    Eigen::Vector2f sizes = m_viewBox.sizes().head<2>();
    float prev = sizes(0) / sizes(1);
    float sum = sizes(0) + sizes(1);

    float h = sum / (1+ratio);
    float w = ratio * h;
    Eigen::Vector2f offset = (sizes - Eigen::Vector2f(w, h)) / 2;

    m_viewBox.min().head<2>() += offset;
    m_viewBox.max().head<2>() -= offset;
}


void OrthographicCamera::translate(const Eigen::Vector2f& offset)
{
    m_viewBox.translate((Eigen::Vector3f() << -offset, 0).finished());
}


void OrthographicCamera::zoom(const Eigen::Vector2f& center, float zoom)
{
    m_viewBox.min().head<2>() = center + (m_viewBox.min().head<2>() - center) / zoom;
    m_viewBox.max().head<2>() = center + (m_viewBox.max().head<2>() - center) / zoom;
}


void OrthographicCamera::normalizedTranslate(const Eigen::Vector2f& offset)
{
    float rml = m_viewBox.max()(0) - m_viewBox.min()(0);
    float tmb = m_viewBox.max()(1) - m_viewBox.min()(1);

    translate(Eigen::Vector2f(
                offset.x() * rml / 2.f,
                offset.y() * tmb / 2.f));
}


void OrthographicCamera::normalizedZoom(const Eigen::Vector2f& center, float zoom)
{
    this->zoom(normalizedToCamera(center), zoom);
}


Eigen::Matrix4f OrthographicCamera::projectionMatrix() const
{
    float rpl = m_viewBox.max()(0) + m_viewBox.min()(0);
    float tpb = m_viewBox.max()(1) + m_viewBox.min()(1);
    float fpn = m_viewBox.max()(2) + m_viewBox.min()(2);

    float rml = m_viewBox.max()(0) - m_viewBox.min()(0);
    float tmb = m_viewBox.max()(1) - m_viewBox.min()(1);
    float fmn = m_viewBox.max()(2) - m_viewBox.min()(2);

    Eigen::Matrix4f m;
    m << 2.f/rml,       0,        0, -rpl/rml,
               0, 2.f/tmb,        0, -tpb/tmb,
               0,       0, -2.f/fmn,  fpn/fmn,
               0,       0,        0,        1;

    return m;
}
