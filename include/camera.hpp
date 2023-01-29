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
    void orbitSetTarget(Vector3f target);
    void orbitSetDistance(float distance);
    void orbitSetTheta(float theta);
    void orbitSetPhi(float phi);
    const Matrix4f view() const;
    const Matrix4f getTransform(Vector2f viewSize) const;
};
