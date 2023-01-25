#pragma once

#include <Eigen/Dense>

using namespace Eigen;

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

    const Matrix4f matrix();
};
