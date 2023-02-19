#pragma once

#include <extmath.hpp>

// Physics body component
//  controls: Model
struct PhysicsBody
{
    static constexpr std::size_t page_size = 65536u;
    Vector3f pos = Vector3f::Zero();
    Vector3f vel = Vector3f::Zero();
    Vector3f acc = Vector3f::Zero();
};

// Debug draw ray component
//  controls: RayTransform
//  sibling: DebugColor
struct DebugRay
{
    Vector3f pos = Vector3f::Zero();
    Vector3f rot = Vector3f::Zero();
    float length = 1.0f;

    const Matrix4f transform() const;
};

struct RayTransform
{
    static constexpr std::size_t page_size = 65536u;
    Matrix4f transform;
};

struct DebugColor
{
    static constexpr std::size_t page_size = 65536u;
    Vector4f color;
};

class ColliderInteriorBox : public AABB
{
public:
    using AABB::AABB;

    bool collide(PhysicsBody& body) const;
};