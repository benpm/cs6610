#include <camera.hpp>

void Camera::orbit() {
    this->pos = direction({this->theta, this->phi, 0.0f}) * distance;
    this->rot = {this->theta, -this->phi, 0.0f};
}

void Camera::orbitPanStart() {
    this->panStartTheta = this->theta;
    this->panStartPhi = this->phi;
}

void Camera::orbitPan(Vector2f delta) {
    this->theta = this->panStartTheta + delta.y();
    this->phi = this->panStartPhi + delta.x();
    this->orbit();
}

void Camera::orbitTarget(Vector3f target)
{
    this->mode = Mode::orbit;
    this->target = target;
    this->orbit();
}

const Vector3f& Camera::orbitTarget() const {
    return this->target;
};

void Camera::orbitDist(float distance) {
    this->mode = Mode::orbit;
    this->distance = distance;
    this->orbit();
}

float Camera::orbitDist() const {
    return this->distance;
};

void Camera::orbitTheta(float theta) {
    this->mode = Mode::orbit;
    this->theta = theta;
    this->orbit();
}

float Camera::orbitTheta() const {
    return this->theta;
};

void Camera::orbitPhi(float phi) {
    this->mode = Mode::orbit;
    this->phi = phi;
    this->orbit();
}

float Camera::orbitPhi() const {
    return this->phi;
};

const Matrix4f Camera::view() const {
    return identityTransform()
        .rotate(euler(rot))
        .translate(-pos)
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