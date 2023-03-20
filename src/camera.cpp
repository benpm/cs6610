#include <camera.hpp>
#include <spdlog/spdlog.h>
#include <model.hpp>
#include <light.hpp>

void CameraControl::orbit() {
    this->mode = Mode::orbit;
    this->pos = this->target + spherePoint(this->phi, this->theta) * distance;
    this->rot = {this->theta, -this->phi, 0.0f};
}

void CameraControl::dragStart() {
    switch (this->mode) {
        case Mode::orbit:
            this->orbitPanStart();
            break;
        default:
            this->posStart = this->pos;
            break;
    }
}

void CameraControl::orbitPanStart() {
    this->panStartTheta = this->theta;
    this->panStartPhi = this->phi;
}

void CameraControl::orbitPan(Vector2f delta) {
    this->theta = std::clamp(this->panStartTheta + delta.y(), -tau4, tau4);
    this->phi = this->panStartPhi + delta.x();
    this->orbit();
}

void CameraControl::orbitTarget(Vector3f target) {
    this->target = target;
    this->orbit();
}
const Vector3f& CameraControl::orbitTarget() const {
    return this->target;
}

void CameraControl::orbitDist(float distance) {
    this->distance = distance;
    this->orbit();
}
float CameraControl::orbitDist() const {
    return this->distance;
}

void CameraControl::orbitTheta(float theta) {
    this->theta = theta;
    this->orbit();
}
float CameraControl::orbitTheta() const {
    return this->theta;
}

void CameraControl::orbitPhi(float phi) {
    this->phi = phi;
    this->orbit();
}
float CameraControl::orbitPhi() const {
    return this->phi;
}

void CameraControl::flyDir(const Vector3f& dir) {
    const Matrix4f view = identityTransform()
        .rotate(euler(this->rot))
        .translate(-this->pos)
        .matrix();
    this->pos += view.block<3, 3>(0, 0).transpose() * dir;
}

void CameraControl::control(const Vector2f& rotateDelta, const Vector2f& dragDelta, const Vector2f& moveDelta) {
    switch (this->mode) {
        case Mode::fly:
            this->flyDir(Vector3f(moveDelta.x(), 0.0f, -moveDelta.y()));
            this->rot += Vector3f(rotateDelta.y(), rotateDelta.x(), 0.0f);
            break;
        case Mode::orbit:
            this->orbitPan({dragDelta.x(), dragDelta.y()});
            break;
        case Mode::trackball:
            break;
        case Mode::track2D:
            this->pos = this->posStart + vec3((dragDelta * 100000.0f) / this->zoom);
            this->pos.z() = 50.0f;
            break;
        default:
            break;
    }
}

void CameraControl::update(Camera& cam) const {
    cam.pos = this->pos;
    cam.rot = this->rot;
    cam.zoom = this->zoom;
}

void CameraControl::update(Model& model) const {
    model.pos = this->pos;
    model.rot = this->rot;
}

void CameraControl::update(Light& light) const {
    light.pos = this->pos;
    light.dir = -direction(this->rot);
}

void CameraControl::universalZoom(float delta) {
    this->orbitDist(this->orbitDist() * (1.0f + delta));
    this->zoom *= (1.0f - delta);
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
            return perspective(this->fov, viewsize.x() / viewsize.y(), this->near, this->far);
        case Projection::orthographic:
            return orthographic(viewsize / this->zoom, this->near, this->far);
        default:
            return Matrix4f::Identity();
    }
}

Vector3f Camera::toView(const Vector3f &point) const {
    return (this->getView() * Vector4f(point.x(), point.y(), point.z(), 1.0f)).head<3>();
}

void Camera::from(const Camera& other) {
    this->pos = other.pos;
    this->rot = other.rot;
    this->fov = other.fov;
    this->zoom = other.zoom;
    this->projection = other.projection;
    this->near = other.near;
    this->far = other.far;
}

Ray Camera::getRay(const Vector2f& screenPoint, const Vector2f& viewSize) const {
    // https://antongerdelan.net/opengl/raycasting.html

    const Vector4f rayClip(
        (screenPoint.x() / viewSize.x()) * 2.0f - 1.0f,
        1.0f - (screenPoint.y() / viewSize.y()) * 2.0f,
        -1.0f,
        1.0f
    );
    const Vector4f rayEye = this->getProj(viewSize).inverse() * rayClip;
    const Vector3f rayWorld = (this->getView().inverse() * vec4(rayEye.head<2>(), -1.0f, 0.0f)).head<3>().normalized();
    return Ray(this->pos, rayWorld);
}