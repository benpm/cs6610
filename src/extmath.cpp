#include <extmath.hpp>

/* Random Number Generator Class */
RNG::RNG(uint seed) : seed(seed), gen(seed) {
}
int RNG::range(int a, int b) {
    const std::uniform_int_distribution<int>::param_type params(a, b);
    return this->idist(gen, params);
}
float RNG::range(float a, float b) {
    const std::uniform_real_distribution<float>::param_type params(a, b);
    return this->rdist(gen, params);
}
Vector3f RNG::vec(const Vector3f& min, const Vector3f& max) {
    return {this->range(min.x(), max.x()), this->range(min.y(), max.y()), this->range(min.z(), max.z())};
}
Vector3f RNG::vec(const Vector3f& max) {
    return this->vec(Vector3f::Zero(), max);
}
Vector3f RNG::rotation()
{
    return {this->range(0.0f, tau), this->range(0.0f, tau), this->range(0.0f, tau)};
}
bool RNG::test(float probability) {
    const std::uniform_real_distribution<float>::param_type params(0, 1);
    return this->rdist(gen, params) < probability;
}

float angle2D(const Vector2f& v) {
    return std::atan2(v.y(), v.x()) + tau2;
}

float degrees(float radians) {
    return radians * 180.0f / tau2;
}

Vector3f hsvToRgb(const Vector3f& hsv) {
    const float h = hsv.x();
    const float s = hsv.y();
    const float v = hsv.z();

    const float c = v * s;
    const float x = c * (1.0f - std::abs(std::fmod(h / 60.0f, 2.0f) - 1.0f));
    const float m = v - c;

    Vector3f rgb;
    if (h < 60.0f) {
        rgb = {c, x, 0.0f};
    } else if (h < 120.0f) {
        rgb = {x, c, 0.0f};
    } else if (h < 180.0f) {
        rgb = {0.0f, c, x};
    } else if (h < 240.0f) {
        rgb = {0.0f, x, c};
    } else if (h < 300.0f) {
        rgb = {x, 0.0f, c};
    } else {
        rgb = {c, 0.0f, x};
    }

    return rgb + Vector3f(m, m, m);
}

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

Vector3f spherePoint(float phi, float theta) {
    return {
        std::cos(theta) * std::sin(phi),
        std::sin(theta),
        std::cos(theta) * std::cos(phi),
    };
}

Vector3f towards(const Vector3f &a, const Vector3f &b) {
    // Euler angles of the rotation from a to b
    return {
        std::atan2(b.z() - a.z(), b.x() - a.x()),
        tau4 * 3.0f + (float)std::atan2(b.y() - a.y(), std::sqrt(std::pow(b.x() - a.x(), 2) + std::pow(b.z() - a.z(), 2))),
        0.0f
    };
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