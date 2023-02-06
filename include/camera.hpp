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

    // Computes camera position and rotation as an orbit camera (uses theta, phi, distance, target)
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

    // Orbit camera: start orbit panning (should be called on click)
    void orbitPanStart();
    // Orbit camera: pan (should be called on drag)
    void orbitPan(Vector2f delta);
    // Orbit camera: set target point
    void orbitTarget(Vector3f target);
    // Orbit camera: get target point
    const Vector3f& orbitTarget() const;
    // Orbit camera: set camera distance to target
    void orbitDist(float distance);
    // Orbit camera: get camera distance to target
    float orbitDist() const;
    // Orbit camera: set camera theta (vertical angle)
    void orbitTheta(float theta);
    // Orbit camera: get camera theta (vertical angle)
    float orbitTheta() const;
    // Orbit camera: set camera phi (horizontal angle)
    void orbitPhi(float phi);
    // Orbit camera: get camera phi (horizontal angle)
    float orbitPhi() const;
    // Modifies "zoom", which does something slightly different depending on the projection
    void universalZoom(float delta);
    // Returns view transformation matrix
    const Matrix4f getView() const;
    // Returns projection transformation matrix
    const Matrix4f getProj(Vector2f viewSize) const;
    // Transforms given point to view space
    Vector3f toView(const Vector3f& point) const;
};
