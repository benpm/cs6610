#include <filesystem>
#include <extmath.hpp>
#include <spdlog/spdlog.h>
#include <glad/glad.h>

void glCheckError_(const char *file, int line) {
    GLenum errorCode;
    while ((errorCode = glGetError()) != GL_NO_ERROR) {
        std::string error;
        switch (errorCode) {
            case GL_INVALID_ENUM:                  error = "INVALID_ENUM"; break;
            case GL_INVALID_VALUE:                 error = "INVALID_VALUE"; break;
            case GL_INVALID_OPERATION:             error = "INVALID_OPERATION"; break;
            case GL_STACK_OVERFLOW:                error = "STACK_OVERFLOW"; break;
            case GL_STACK_UNDERFLOW:               error = "STACK_UNDERFLOW"; break;
            case GL_OUT_OF_MEMORY:                 error = "OUT_OF_MEMORY"; break;
            case GL_INVALID_FRAMEBUFFER_OPERATION: error = "INVALID_FRAMEBUFFER_OPERATION"; break;
        }
        spdlog::error("GL_{} - {}:{}", error, std::filesystem::path(file).filename().string(), line);
    }
}

/* Random Number Generator Class */
RNG::RNG(uint32_t seed) : seed(seed), gen(seed) {
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
Vector3f RNG::vec(const AABB& bounds) {
    return this->vec(bounds.min, bounds.max);
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

Vector3f spherePoint(const Vector2f& point) {
    return spherePoint(point.x(), point.y());
}

Vector2f pointSphere(const Vector3f& point) {
    return {
        std::atan2(point.z(), point.x()) + tau2,
        std::asin(point.y())
    };
}

Vector3f rotate(const Vector3f &v, const Vector3f &axisAngles) {
    return euler(axisAngles) * v;
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

Vector3f vec3(const Vector2f& v, float z) {
    return {v.x(), v.y(), z};
}

Vector3f vec3(float xyz) {
    return {xyz, xyz, xyz};
}

Vector2f vec2(const Vector3f &v) {
    return {v.x(), v.y()};
}

AABB::AABB(const Vector3f& min, const Vector3f& max)
    : min(min), max(max) {}

std::array<Vector3f, 4> AABB::cornersXY() const {
    return {
        Vector3f(min.x(), min.y(), 0.0f),
        Vector3f(max.x(), min.y(), 0.0f),
        Vector3f(max.x(), max.y(), 0.0f),
        Vector3f(min.x(), max.y(), 0.0f),
    };
}

float AABB::width() const {
    return max.x() - min.x();
}

void AABB::width(float width) {
    const Vector3f c = this->center();
    this->place(c, Vector3f(width, this->height(), this->depth()));
}

float AABB::height() const {
    return max.y() - min.y();
}

void AABB::height(float height) {
    const Vector3f c = this->center();
    this->place(c, Vector3f(this->width(), height, this->depth()));
}

float AABB::depth() const {
    return max.z() - min.z();
}

void AABB::depth(float depth) {
    const Vector3f c = this->center();
    this->place(c, Vector3f(this->width(), this->height(), depth));
}

void AABB::size(float size) {
    this->place(this->center(), Vector3f(size, size, size));
}

Vector3f AABB::center() const {
    return (min + max) / 2.0f;
}

void AABB::place(const Vector3f& center, const Vector3f& size) {
    this->min = (center - size) / 2.0f;
    this->max = (center + size) / 2.0f;
}

void AABB::place(const Vector2f& center, const Vector2f& size) {
    this->min = vec3((center - size) / 2.0f, this->min.z());
    this->max = vec3((center + size) / 2.0f, this->max.z());
}
