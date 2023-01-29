#include <extmath.hpp>

Transform3f identityTransform() {
    return Transform3f::Identity();
}

Vector3f toEigen(const cy::Vec3f& vec) {
    return Vector3f(vec.x, vec.y, vec.z);
}

Quaternionf euler(const Vector3f& axisAngles) {
    return 
        AngleAxisf(axisAngles.x(), Vector3f::UnitX()) *
        AngleAxisf(axisAngles.y(), Vector3f::UnitY()) *
        AngleAxisf(axisAngles.z(), Vector3f::UnitZ());
}

Vector3f direction(const Vector3f &axisAngles) {
    return {
        std::cos(axisAngles.y()) * std::cos(axisAngles.x()),
        std::cos(axisAngles.y()) * std::sin(axisAngles.x()),
        std::sin(axisAngles.y())
    };
}

Vector3f towards(const Vector3f &a, const Vector3f &b) {
    // Euler angles of the rotation from a to b
    return {
        std::atan2(b.z() - a.z(), b.x() - a.x()),
        tau4 * 3.0f + std::atan2(b.y() - a.y(), std::sqrt(std::pow(b.x() - a.x(), 2) + std::pow(b.z() - a.z(), 2))),
        0.0f
    };
}

Matrix4f lookAt(const Vector3f& eye, const Vector3f& center, const Vector3f& up) {
    const Vector3f zAxis = (center - eye).normalized();
    const Vector3f xAxis = zAxis.cross(up).normalized();
    const Vector3f yAxis = xAxis.cross(zAxis);

    Matrix4f m = Matrix4f::Identity();
    m(0, 0) = xAxis.x();
    m(1, 0) = yAxis.x();
    m(2, 0) = zAxis.x();
    m(0, 1) = yAxis.y();
    m(1, 1) = yAxis.y();
    m(2, 1) = zAxis.y();
    m(0, 2) = zAxis.z();
    m(1, 2) = yAxis.z();
    m(2, 2) = zAxis.z();
    m(0, 3) = -xAxis.dot(eye);
    m(1, 3) = -yAxis.dot(eye);
    m(2, 3) = -zAxis.dot(eye);
    return m;
}

Matrix4f perspective(float fov, float aspect, float near, float far) {
    Matrix4f m = Matrix4f::Zero();
    m(0, 0) = 1.0f / (aspect * tanf(fov / 2.0f));
    m(1, 1) = 1.0f / tanf(fov / 2.0f);
    m(2, 2) = -(far + near) / (far - near);
    m(2, 3) = -2.0f * far * near / (far - near);
    m(3, 2) = -1.0f;
    return m;
}

Matrix4f orthographic(const Vector2f& size, float near, float far) {
    Matrix4f m = Matrix4f::Identity();
    m(0, 0) = 2.0f / size.x();
    m(1, 1) = 2.0f / size.y();
    m(2, 2) = -2.0f / (far - near);
    m(3, 2) = -(far + near) / (far - near);
    return m;
}