#include <camera.hpp>

void Camera::orbit() {
    this->pos = direction({this->theta, this->phi, 0.0f}) * distance;
    this->rot = {-this->theta, this->phi + tau4, 0.0f};
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

const Matrix4f Camera::view() const {
    return identityTransform()
        .rotate(euler(rot))
        .translate(pos)
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