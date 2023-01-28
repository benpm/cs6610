#include <extmath.hpp>

Matrix4f eulerRot(const Vector3f& euler) {
    return Transform(Affine3f(
        AngleAxisf(euler.x(), Vector3f::UnitX()) *
        AngleAxisf(euler.y(), Vector3f::UnitY()) *
        AngleAxisf(euler.z(), Vector3f::UnitZ())
    )).matrix();
}

Matrix4f lookAt(const Vector3f& eye, const Vector3f& center, const Vector3f& up) {
    const Vector3f f = (center - eye).normalized();
    const Vector3f s = f.cross(up).normalized();
    const Vector3f u = s.cross(f);

    Matrix4f m = Matrix4f::Identity();
    m(0, 0) = s.x();
    m(1, 0) = s.y();
    m(2, 0) = s.z();
    m(0, 1) = u.x();
    m(1, 1) = u.y();
    m(2, 1) = u.z();
    m(0, 2) = -f.x();
    m(1, 2) = -f.y();
    m(2, 2) = -f.z();
    m(3, 0) = -s.dot(eye);
    m(3, 1) = -u.dot(eye);
    m(3, 2) = f.dot(eye);
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