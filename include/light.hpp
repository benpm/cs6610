#pragma once

#include <extmath.hpp>

class Camera;

class Light
{
public:
    // Position of light in world space if point, direction of light in world space if directional
    Vector3f pos = {0.0f, 0.0f, 0.0f};
    Vector3f color = {1.0f, 1.0f, 1.0f};
    float intensity = 10.0f;
    LightType type = LightType::point;

    Light(const Vector3f& pos, const Vector3f& color, float intensity, LightType type);

    uLight toStruct(const Camera& camera) const;
};