/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _EXAMPLES_VITELOTTE_COMMON_TRACKBALL_
#define _EXAMPLES_VITELOTTE_COMMON_TRACKBALL_


#include <Eigen/Geometry>


Eigen::Matrix4f orthographicProjection(
        float l, float r, float b, float t, float n, float f);

Eigen::Matrix4f orthographicProjection(
        const Eigen::AlignedBox3f& viewBox);

Eigen::Matrix4f perspectiveProjection(
        float l, float r, float b, float t, float n, float f);

Eigen::Matrix4f perspectiveProjection(
        const Eigen::AlignedBox3f& viewBox, float dist = 1);

class Trackball {
public:
    enum State {
        Idle, Rotating, Translating
    };


public:
    Trackball();

    void updateCamera();

    /// \brief The center of the object to look at.
    const Eigen::Vector3f& sceneCenter() const;
    /// \brief The distance between `center` and the camera.
    float sceneDistance() const;
    /// \brief The radius of the object, define the fov.
    float sceneRadius() const;
    /// \brief The orientation of the camera.
    const Eigen::Quaternionf& sceneOrientation() const;
    /// \brief The viewport box in screen-space.
    const Eigen::AlignedBox2f& screenViewport() const;
    float minNear() const;
    float nearOffset() const;
    float farOffset() const;

    float minScreenViewportSize() const;
    bool isOrthographic() const;
    bool isPerspective() const;

    Eigen::Matrix4f computeViewMatrix() const;
    Eigen::Matrix4f computeProjectionMatrix() const;

    void setSceneCenter(const Eigen::Vector3f& scnCenter);
    void setSceneDistance(float scnDistance);
    void setSceneRadius(float scnRadius);
    void setSceneOrientation(const Eigen::Quaternionf& scnOrientation);
    void setScreenViewport(const Eigen::AlignedBox2f& scnViewport);
    void setMinNear(float minNear);
    void setNearFarOffsets(float nearOffset, float farOffset);

    State state() const;
    bool isIdle() const;

    /// \brief Rotate around `center`.
    void rotate(const Eigen::Quaternionf& rot);
    bool isRotating() const;
    void startRotation(const Eigen::Vector2f& scrPos);
    void dragRotate(const Eigen::Vector2f& scrPos);
    void cancelRotation();
    void endRotation();

    /// \brief Translate in the view plane.
    void translate(const Eigen::Vector2f& scnVec);
    void translate(const Eigen::Vector3f& scnVec);
    bool isTranslating() const;
    void startTranslation(const Eigen::Vector2f& scrPos);
    void dragTranslate(const Eigen::Vector2f& scrPos);
    void cancelTranslation();
    void endTranslation();

    /// \brief Move camera forward or backward without changing fov. (So it
    /// changes radius.)
    void zoom(float factor);

    /// \brief Grow or shrink radius, changing the fov.
    void grow(float factor);

    /// \brief Do a dolly zoom: move the camera and change the fov so that the
    /// subject in the same frame.
    void dollyZoom(float factor);

private:
    Eigen::Vector2f normFromScr(const Eigen::Vector2f& scrPos) const;

    Eigen::Quaternionf computeRotation(const Eigen::Vector2f& scrPos) const;
    Eigen::Vector3f computeTranslation(const Eigen::Vector2f& scrPos) const;

private:
    Eigen::Vector3f _scnCenter;
    float _scnDistance;
    float _scnRadius;
    Eigen::Quaternionf _scnOrientation;
    Eigen::AlignedBox2f _scrViewport;
    float _minNear;
    float _nearOffset;
    float _farOffset;

    State _state;
    Eigen::Vector2f _scrMouseInit;
    Eigen::Quaternionf _scnOrientInit;
    Eigen::Vector3f _scnCenterInit;

};


#endif
