#include <camera.hpp>

Camera::Camera(Vector3f pos, Vector3f rot, float near, float far, float fov) :
    pos(pos), rot(rot), near(near), far(far), fov(fov) {}

const Matrix4f Camera::view() const {
    return identityTransform()
        .rotate(euler(rot))
        .translate(pos)
        .inverse()
        .matrix();
}

const Matrix4f Camera::transform(Vector2f viewsize) const {
    return perspective(fov, viewsize.x() / viewsize.y(), near, far) * this->view();
}