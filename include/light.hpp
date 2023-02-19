#pragma once

#include <extmath.hpp>

class Camera;

enum class LightType: uint32_t {
    point,
    directional
};

// Light struct for use in shaders: exists in view-space
struct uLight {
    alignas(16) Vector3f position;
    alignas(16) Vector3f color;
    float intensity;
    LightType type;
};

// Light component for manipulation
class Light
{
public:
    // Position of light in world space if point, direction of light in world space if directional
    Vector3f pos = {0.0f, 0.0f, 0.0f};
    Vector3f color = {1.0f, 1.0f, 1.0f};
    float intensity = 10.0f;
    LightType type = LightType::point;

    Light() = default;
    Light(const Vector3f& pos, const Vector3f& color, float intensity, LightType type);

    // Returns a uLight struct in the given camera's coordinate space
    uLight toStruct(const Camera& camera) const;
};