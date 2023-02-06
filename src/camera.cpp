#include <camera.hpp>

void Camera::orbit() {
    this->mode = Mode::orbit;
    this->pos = this->target + spherePoint(this->phi, this->theta) * distance;
    this->rot = {this->theta, -this->phi, 0.0f};
}

void Camera::orbitPanStart() {
    this->panStartTheta = this->theta;
    this->panStartPhi = this->phi;
}

void Camera::orbitPan(Vector2f delta) {
    this->theta = std::clamp(this->panStartTheta + delta.y(), -tau4, tau4);
    this->phi = this->panStartPhi + delta.x();
    this->orbit();
}

void Camera::orbitTarget(Vector3f target) {
    this->target = target;
    this->orbit();
}
const Vector3f& Camera::orbitTarget() const {
    return this->target;
}

void Camera::orbitDist(float distance) {
    this->distance = distance;
    this->orbit();
}
float Camera::orbitDist() const {
    return this->distance;
}

void Camera::orbitTheta(float theta) {
    this->theta = theta;
    this->orbit();
}
float Camera::orbitTheta() const {
    return this->theta;
}

void Camera::orbitPhi(float phi) {
    this->phi = phi;
    this->orbit();
}
float Camera::orbitPhi() const {
    return this->phi;
}

void Camera::universalZoom(float delta) {
    switch (this->projection) {
        case Projection::perspective:
            this->orbitDist(this->orbitDist() * (1.0f + delta));
            break;
        case Projection::orthographic:
            this->zoom *= (1.0f - delta);
            break;
        default:
            break;
    }
}

const Matrix4f Camera::getView() const {
    return identityTransform()
        .rotate(euler(rot))
        .translate(-pos)
        .matrix();
}

const Matrix4f Camera::getProj(Vector2f viewsize) const {
    switch (this->projection) {
        case Projection::perspective:
            return perspective(fov, viewsize.x() / viewsize.y(), this->near, this->far);
        case Projection::orthographic:
            return orthographic(viewsize / this->zoom, this->near, this->far);
        default:
            return Matrix4f::Identity();
    }
}

Vector3f Camera::toView(const Vector3f &point) const {
    return (this->getView() * Vector4f(point.x(), point.y(), point.z(), 1.0f)).head<3>();
}
