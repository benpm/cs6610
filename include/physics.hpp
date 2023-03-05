#pragma once

#include <extmath.hpp>

// COMPONENT: Physics body
//  - controls: Model
struct PhysicsBody
{
    static constexpr std::size_t page_size = 65536u;
    float mass = 1.0f;
    Vector3f pos = Vector3f::Zero();
    Vector3f vel = Vector3f::Zero();
    Vector3f acc = Vector3f::Zero();
};

// COMPONENT: Debug draw ray
//  - controls: RayTransform
//  - sibling: DebugColor
struct DebugRay
{
    Vector3f pos = Vector3f::Zero();
    Vector3f rot = Vector3f::Zero();
    float length = 1.0f;

    const Matrix4f transform() const;
};

// COMPONENT: Box collider
struct ColliderBox
{
    static constexpr std::size_t page_size = 65536u;

    Vector3f halfExtents = Vector3f::Zero();
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

// COMPONENT: Rigid body
//  - controls: PhysicsBody
//  - sibling: DebugColor, BoxCollider
struct RigidBody
{
    static constexpr std::size_t page_size = 65536u;

    RigidBody(const ColliderBox& collider);

    Vector3f angMomentum = Vector3f::Zero();
    Matrix3f rot = euler(Vector3f::Zero()).toRotationMatrix();
    // Rest inertial tensor
    Matrix3f J;
    // Inverse of rest inertial tensor
    Matrix3f invJ;
};

namespace Physics {
    void simulate(float dt, RigidBody& rb, PhysicsBody& b);
};