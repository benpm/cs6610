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
        spdlog::error("GL_{} - {}:{}", error, fs::path(file).filename().string(), line);
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

Vector2f pointSphere(const Vector3f& p) {
    return {
        std::atan2(p.y(), std::sqrt(p.x()*p.x() + p.z()*p.z())),
        std::atan2(p.x(), p.z()) + tau4
    };
}

Vector3f dirToRot(const Vector3f& dir) {
    return Quaternionf::FromTwoVectors(-Vector3f::UnitZ(), dir)
        .toRotationMatrix().eulerAngles(0, 1, 2);
}

Matrix3f skew(const Vector3f& v) {
    Matrix3f m; m <<
        0.0f, -v.z(), v.y(),
        v.z(), 0.0f, -v.x(),
        -v.y(), v.x(), 0.0f;
    return m;
}

Vector3f rotate(const Vector3f &v, const Vector3f &axisAngles) {
    return euler(axisAngles) * v;
}

Vector3f direction(const Vector3f& axisAngles) {
    // Rotation about x axis is up/down, rotation about y axis is left/right around sphere
    return spherePoint({axisAngles.y(), axisAngles.x()});
}

Vector3f towards(const Vector3f &a, const Vector3f &b) {
    return dirToRot(b - a);
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

Matrix4f perspective(const Vector4f& view, float near, float far) {
    Matrix4f m = Matrix4f::Zero();
    m(0, 0) = 2.0f * near / (view.z() - view.x());
    m(1, 1) = 2.0f * near / (view.w() - view.y());
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

Vector3f project(const Vector3f& a, const Vector3f& b) {
    return a.dot(b) / a.dot(a) * a;
}

Matrix4f transform(const Vector3f& translation, const Vector3f& axisAngles, const Vector3f& scale) {
    return identityTransform()
        .translate(translation)
        .rotate(euler(axisAngles))
        .scale(scale)
        .matrix();
}

Matrix4f transform(const Vector3f& translation, const Matrix3f& rotMatrix, const Vector3f& scale) {
    return identityTransform()
        .translate(translation)
        .rotate(rotMatrix)
        .scale(scale)
        .matrix();
}

Vector3f transformPoint(const Vector3f& point, const Matrix4f& transform) {
    return (transform * vec4(point, 1.0f)).head<3>();
}

Vector3f transformDir(const Vector3f& point, const Matrix4f& transform) {
    return (transform * vec4(point, 0.0f)).head<3>().normalized();
}

Vector3f vec3(float v[3]) {
    return {v[0], v[1], v[2]};
}

Vector3f vec3(const Vector2f &v, float z) {
    return {v.x(), v.y(), z};
}

Vector3f vec3(float xyz) {
    return {xyz, xyz, xyz};
}

Vector2f vec2(const Vector3f &v) {
    return {v.x(), v.y()};
}

Vector4f vec4(const Vector3f &v, float w) {
    return {v.x(), v.y(), v.z(), w};
}

Vector4f vec4(const Vector2f &v, float z, float w) {
    return {v.x(), v.y(), z, w};
}

// AABB

const std::array<Vector3f, 6u> AABB::faceNormals = {
    Vector3f( 1.0f,  0.0f,  0.0f),
    Vector3f(-1.0f,  0.0f,  0.0f),
    Vector3f( 0.0f,  1.0f,  0.0f),
    Vector3f( 0.0f, -1.0f,  0.0f),
    Vector3f( 0.0f,  0.0f,  1.0f),
    Vector3f( 0.0f,  0.0f, -1.0f)
};

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

std::array<Vector3f, 8> AABB::corners() const {
    return {
        Vector3f(min.x(), min.y(), min.z()),
        Vector3f(max.x(), min.y(), min.z()),
        Vector3f(max.x(), max.y(), min.z()),
        Vector3f(min.x(), max.y(), min.z()),
        Vector3f(min.x(), min.y(), max.z()),
        Vector3f(max.x(), min.y(), max.z()),
        Vector3f(max.x(), max.y(), max.z()),
        Vector3f(min.x(), max.y(), max.z()),
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

Vector3f AABB::extents() const {
    return (max - min) / 2.0f;
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

bool AABB::contains(const Vector3f &point, float eps) const {
    return point.x() - min.x() > -eps && point.x() - max.x() < eps &&
           point.y() - min.y() > -eps && point.y() - max.y() < eps &&
           point.z() - min.z() > -eps && point.z() - max.z() < eps;
}

float AABB::volume() const {
    return this->width() * this->height() * this->depth();
}

std::array<Vector3f, 8> AABB::vertices() const {
    std::array<Vector3f, 8> arr;
    const std::array<const Vector3f*, 2> e {&this->min, &this->max};
    for (int i = 0; i < 8; i++) {
        arr[i] = {e[(i >> 0) & 1]->x(), e[(i >> 1) & 1]->y(), e[(i >> 2) & 1]->z()};
    }
    return arr;
}

std::array<std::pair<Vector3f, Vector3f>, 12> AABB::edges() const {
    const std::array<Vector3f, 8> v = this->vertices();
    return {
        std::pair<Vector3f, Vector3f>(v[0], v[1]),
        std::pair<Vector3f, Vector3f>(v[1], v[2]),
        std::pair<Vector3f, Vector3f>(v[2], v[3]),
        std::pair<Vector3f, Vector3f>(v[3], v[0]),
        std::pair<Vector3f, Vector3f>(v[4], v[5]),
        std::pair<Vector3f, Vector3f>(v[5], v[6]),
        std::pair<Vector3f, Vector3f>(v[6], v[7]),
        std::pair<Vector3f, Vector3f>(v[7], v[4]),
        std::pair<Vector3f, Vector3f>(v[0], v[4]),
        std::pair<Vector3f, Vector3f>(v[1], v[5]),
        std::pair<Vector3f, Vector3f>(v[2], v[6]),
        std::pair<Vector3f, Vector3f>(v[3], v[7]),
    };
}

std::optional<Vector3f> AABB::intersect(const Ray &ray) const {
    const float eps = 1e-6f;

    // ray.direction is unit direction vector of ray
    const Vector3f dirFrac = ray.direction.cwiseInverse();
    // lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
    // r.org is origin of ray
    const float t1 = (this->min.x() - ray.origin.x()) * dirFrac.x();
    const float t2 = (this->max.x() - ray.origin.x()) * dirFrac.x();
    const float t3 = (this->min.y() - ray.origin.y()) * dirFrac.y();
    const float t4 = (this->max.y() - ray.origin.y()) * dirFrac.y();
    const float t5 = (this->min.z() - ray.origin.z()) * dirFrac.z();
    const float t6 = (this->max.z() - ray.origin.z()) * dirFrac.z();

    const float tmin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
    const float tmax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));

    if (tmax < eps || tmin - tmax > eps) {
        return std::nullopt;
    }

    return ray.origin + ray.direction * tmin;
}

AABB AABB::operator*(const Vector3f& v) const {
    return AABB(Vector3f(this->min.cwiseProduct(v)), Vector3f(this->max.cwiseProduct(v)));
}

Plane::Plane(const Vector3f& origin, const Vector3f& normal)
    : origin(origin), normal(normal) {}

Ray::Ray(const Vector3f& origin, const Vector3f& direction)
    : origin(origin), direction(direction) {}

std::optional<Vector3f> Ray::intersect(const Plane& plane) const {
    const float d = plane.normal.dot(this->direction);
    if (d < 1e-6f) {
        return this->origin;
    }
    const float t = (plane.origin - this->origin).dot(plane.normal) / d;
    if (t < 0.0f) {
        return std::nullopt;
    }
    return this->origin + this->direction * t;
}

std::optional<Vector3f> rayTriangleIntersect(
    const Ray& ray,
    const Vector3f &v0, const Vector3f &v1, const Vector3f &v2)
{
    const Vector3f v0v1 = v1 - v0;
    const Vector3f v0v2 = v2 - v0;
    const Vector3f pvec = ray.direction.cross(v0v2);
    const float det = v0v1.dot(pvec);
    if (det < 1e-6) return std::nullopt;

    const float invDet = 1 / det;

    const Vector3f tvec = ray.origin - v0;
    const float u = tvec.dot(pvec) * invDet;
    if (u < 0 || u > 1) return std::nullopt;

    const Vector3f qvec = tvec.cross(v0v1);
    const float v = ray.direction.dot(qvec) * invDet;
    if (v < 0 || u + v > 1) return std::nullopt;
    
    const float t = v0v2.dot(qvec) * invDet;
    return ray.origin + ray.direction * t;
}

std::optional<Vector3f> Ray::intersect(const Triangle & tri) const {
    return rayTriangleIntersect(*this, tri.verts[0], tri.verts[1], tri.verts[2]);
}

Ray Ray::transformed(const Matrix4f &transform) const {
    return Ray(
        (transform * vec4(this->origin, 1.0f)).head<3>(),
        (transform * vec4(this->direction, 0.0f)).head<3>().normalized());
}

uint64_t cantor(uint32_t x, uint32_t y) {
    return ((x + y) * (x + y + 1u)) / 2u + y;
}