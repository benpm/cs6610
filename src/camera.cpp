#include <camera.hpp>

Camera::Camera(Vector3f pos, Vector3f rot, float near, float far, float fov) :
    pos(pos), rot(rot), near(near), far(far), fov(fov) {}

const Matrix4f Camera::view() const {
    return lookAt(pos, pos + rot, {0.0f, 1.0f, 0.0f});
}

const Matrix4f Camera::matrix(Vector2f viewsize) const {
    return this->view() * perspective(fov, viewsize.x() / viewsize.y(), near, far);
}