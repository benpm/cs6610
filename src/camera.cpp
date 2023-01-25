#include <camera.hpp>

Camera::Camera(fvec3 pos, fvec3 dir, float near, float far) :
    pos(pos), dir(dir), near(near), far(far) {}

const mat4x4 Camera::matrix() {
    return 
}