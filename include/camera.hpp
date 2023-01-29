#pragma once

#include <extmath.hpp>

class Camera
{
private:
    Vector3f target = Vector3f::Zero();
    float distance = 1.0f;
    float theta = 0.0f;
    float phi = 0.0f;
    float panStartTheta = 0.0f;
    float panStartPhi = 0.0f;

    void orbit();
public:
    Vector3f pos = Vector3f::Zero();
    Vector3f rot = Vector3f::Zero();
    float near = 0.01f;
    float far = 10000.0f;
    float fov = 1.1f;
    float zoom = 1.0f;
    enum class Projection {
        perspective,
        orthographic
    } projection = Projection::perspective;
    enum class Mode {
        free,
        orbit,
        trackball
    } mode = Mode::free;

    void orbitPanStart();
    void orbitPan(Vector2f delta);
    void orbitTarget(Vector3f target);
    const Vector3f& orbitTarget() const;
    void orbitDist(float distance);
    float orbitDist() const;
    void orbitTheta(float theta);
    float orbitTheta() const;
    void orbitPhi(float phi);
    float orbitPhi() const;
    const Matrix4f view() const;
    const Matrix4f getTransform(Vector2f viewSize) const;
};
