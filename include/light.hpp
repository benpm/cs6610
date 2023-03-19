#pragma once

#include <extmath.hpp>

class Camera;

enum class LightType: uint32_t {
    point,
    directional,
    spot
};

// Light struct for use in shaders
struct uLight {
    alignas(16) Vector3f position;
    alignas(16) Vector3f color;
    alignas(16) Vector3f direction;
    float intensity;
    float range;
    float spotAngle;
    LightType type;
};

// Light component for manipulation
class Light
{
public:
    // Position of light in world space if point (ignored if directional)
    Vector3f pos = {0.0f, 0.0f, 0.0f};
    // Direction of light in world space if spot or directional
    Vector3f dir = {0.0f, 0.0f, 0.0f};
    // Light color
    Vector3f color = {1.0f, 1.0f, 1.0f};
    // Light amount
    float intensity = 1.0f;
    // Light range (point, spot)
    float range = 3.0f;
    // Spotlight angle limit in radians (converted to cos(angle/2) in toStruct)
    float spotAngle = tau4;
    // Type of light
    LightType type = LightType::point;

    Light() = default;
    Light(const Vector3f& pos, const Vector3f& color, float intensity, LightType type);

    // Returns a uLight struct in the given camera's coordinate space
    uLight toStruct(const Camera& camera) const;
};