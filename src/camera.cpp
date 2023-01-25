#include <camera.hpp>

Camera::Camera(Vector3f pos, Vector3f rot, float near, float far, float fov) :
    pos(pos), rot(rot), near(near), far(far), fov(fov) {}

const Matrix4f Camera::matrix() {
    return Matrix4f::Identity();
}