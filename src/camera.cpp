#include <camera.hpp>

void Camera::orbit() {
    this->pos = -direction({this->theta, this->phi, 0.0f}) * distance;
    this->rot = towards(this->pos, this->target);
}

void Camera::orbitSetTarget(Vector3f target) {
    this->mode = Mode::orbit;
    this->target = target;
    this->orbit();
}

void Camera::orbitSetDistance(float distance) {
    this->mode = Mode::orbit;
    this->distance = distance;
    this->orbit();
}

void Camera::orbitSetTheta(float theta) {
    this->mode = Mode::orbit;
    this->theta = theta;
    this->orbit();
}

void Camera::orbitSetPhi(float phi) {
    this->mode = Mode::orbit;
    this->phi = phi;
    this->orbit();
}

void Camera::lookAt(Vector3f target) {
    this->mode = Mode::free;
    this->target = target;
    this->rot = Vector3f(
        std::atan2(this->pos.z() - this->target.z(), this->pos.x() - this->target.x()),
        std::atan2(this->pos.y() - this->target.y(), std::sqrt(std::pow(this->pos.x() - this->target.x(), 2) + std::pow(this->pos.z() - this->target.z(), 2))),
        0.0f
    );
}

const Matrix4f Camera::view() const {
    return identityTransform()
        .rotate(euler(rot))
        .translate(pos)
        .inverse()
        .matrix();
}

const Matrix4f Camera::getTransform(Vector2f viewsize) const {
    switch (this->projection) {
        case Projection::perspective:
            return perspective(fov, viewsize.x() / viewsize.y(), this->near, this->far) * this->view();
        case Projection::orthographic:
            return orthographic(viewsize / this->zoom, this->near, this->far) * this->view();
        default:
            return Matrix4f::Identity();
    }
}