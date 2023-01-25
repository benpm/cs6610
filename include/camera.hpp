#pragma once

#include <glm/glm.hpp>

using namespace glm;

class Camera
{
private:
    fvec3 pos;
    fvec3 dir;
    float near;
    float far;
public:
    Camera(fvec3 pos, fvec3 dir, float near, float far);

    const mat4x4 matrix();
};
