#pragma once

#include <random>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#ifdef PLATFORM_WINDOWS
    #include <cstdint>
#endif
#include <cyVector.h>
#include <spdlog/fmt/bundled/format.h>

using namespace Eigen;

using Transform3f = Transform<float, 3, Affine>;

constexpr float tau = 6.283185307179586476925286766559f;
constexpr float tau2 = tau / 2.0f;
constexpr float tau4 = tau / 4.0f;

constexpr size_t nVertAttribs = 3u;

struct uMaterial {
    alignas(16) Vector3f diffuseColor = {1.0f, 1.0f, 1.0f};
    alignas(16) Vector3f specularColor = {1.0f, 1.0f, 1.0f};
    alignas(16) Vector3f ambientColor = {1.0f, 1.0f, 1.0f};
    float shininess = 35.0f;
    float specularFactor = 2.0f;
    float ambientFactor = 0.05f;
};

struct uLight {
    alignas(16) Vector3f position = {0.0f, 0.0f, 0.0f};
    alignas(16) Vector3f color = {1.0f, 1.0f, 1.0f};
    float intensity = 1.0f;
};

// fmt overload for Matrix4f
template<> struct fmt::formatter<Matrix4f> {
    constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
        return ctx.end();
    }

    template <typename FormatContext>
    auto format(const Matrix4f& input, FormatContext& ctx) -> decltype(ctx.out()) {
        return format_to(ctx.out(),
            "[{}, {}, {}, {}]\n[{}, {}, {}, {}]\n[{}, {}, {}, {}]\n[{}, {}, {}, {}]",
            input(0, 0), input(0, 1), input(0, 2), input(0, 3),
            input(1, 0), input(1, 1), input(1, 2), input(1, 3),
            input(2, 0), input(2, 1), input(2, 2), input(2, 3),
            input(3, 0), input(3, 1), input(3, 2), input(3, 3)
            );
    }
};

// fmt overload for Vector3f
template<> struct fmt::formatter<Vector3f> {
    constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
        return ctx.end();
    }

    template <typename FormatContext>
    auto format(const Vector3f& input, FormatContext& ctx) -> decltype(ctx.out()) {
        return format_to(ctx.out(), "[{:.1f}, {:.1f}, {:.1f}]", input.x(), input.y(), input.z());
    }
};

// fmt overload for std::vector
template <typename T> struct fmt::formatter<std::vector<T>> {
    constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
        return ctx.end();
    }

    template <typename FormatContext>
    auto format(const std::vector<T>& input, FormatContext& ctx) -> decltype(ctx.out()) {
        std::string out = "[";
        for (const auto& item : input) {
            out += fmt::format("{}", item);
            out += ", ";
        }
        out += "]";
        return format_to(ctx.out(), "{}", out);
    }
};

//Random number generation helper class
class RNG
{
public:
    const uint seed;

private:
    std::minstd_rand gen;
    std::uniform_real_distribution<float> rdist;
    std::uniform_int_distribution<int> idist;
    std::uniform_int_distribution<ulong> uldist;
public:
    //Default constructor, uses specified seed for number generation
    RNG(uint seed);
    //Generate a random number between 0 and 1, returns if this number is less than given probability value
    bool test(float probability);
    //Random range from a to b, inclusive
    int range(int a, int b);
    //Random range from a to b, inclusive
    float range(float a, float b);
    //Random vector3
    Vector3f vec(const Vector3f& min, const Vector3f& max);
    //Random vector3 with min 0,0,0
    Vector3f vec(const Vector3f& max);
    //Random euler angles
    Vector3f rotation();
    //Random vector between two vectors
    template <class T> Vector2<T> vecRange(T a, T b) {
        return {this->range(a, b),
                this->range(a, b)};
    };
    //Choose random from list of items
    template <class T> T choose(const std::initializer_list<T> items) {
        auto it = items.begin();
        std::advance(it, range(0, items.size() - 1));
        return *it;
    };
};

// Linear interpolate
template <typename T> T lerp(const T& a, const T& b, float t) {
    return a + (b - a) * t;
}
// Normalized atan2
float angle2D(const Vector2f& v);
// Convert radians to degrees
float degrees(float radians);
/**
 * @brief Convert HSV to RGB
 * 
 * @param hsv hue(0-360deg), saturation(0-1), value(0-1)
 * @return RGB
 */
Vector3f hsvToRgb(const Vector3f& hsv);
// Return a new identity transform
Transform3f identityTransform();
// Convert cy::Vec3f to Vector3f
Vector3f toEigen(const cy::Vec3f& vec);
// Euler angles to quaternion
Quaternionf euler(const Vector3f& axisAngles);
/**
 * @brief Convert spherical to cartesian coordinates (+Y up) ...
 *        (theta=0, phi=0) -> (x=0, y=0, z=1)
 * 
 *        distance from origin is always 1
 * 
 * @param phi Horizontal angle
 * @param theta Vertical angle
 * @return Cartesian coordinate
 */
Vector3f spherePoint(float phi, float theta);
// Angle from a to b
Vector3f towards(const Vector3f& a, const Vector3f& b);
// Perspective projection matrix
Matrix4f perspective(float fov, float aspect, float near, float far);
// Orthographic projection matrix
Matrix4f orthographic(const Vector2f& size, float near, float far);