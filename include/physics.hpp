#pragma once

#include <extmath.hpp>

struct PhysicsBody
{
    Vector3f pos = Vector3f::Zero();
    Vector3f vel = Vector3f::Zero();
    Vector3f acc = Vector3f::Zero();
};

struct DebugRay
{
    Vector3f pos = Vector3f::Zero();
    Vector3f rot = Vector3f::Zero();
    float length = 1.0f;

    const Matrix4f transform() const;
};

struct RayTransform
{
    static constexpr auto in_place_delete = true;
    Matrix4f transform;
};

struct DebugColor
{
    static constexpr auto in_place_delete = true;
    Vector4f color;
};