#pragma once

#include <extmath.hpp>

class Camera
{
private:
    Vector3f pos;
    Vector3f rot;
    float near;
    float far;
    float fov;
public:
    Camera(Vector3f pos, Vector3f rot, float near, float far, float fov);

    const Matrix4f view() const;
    const Matrix4f projection(Vector2f viewSize) const;
};
