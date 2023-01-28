#pragma once

#include <extmath.hpp>

class Camera
{
private:
public:
    Vector3f pos;
    Vector3f rot;
    float near;
    float far;
    float fov;

    Camera(Vector3f pos, Vector3f rot, float near, float far, float fov);

    const Matrix4f view() const;
    const Matrix4f transform(Vector2f viewSize) const;
};
