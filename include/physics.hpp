#pragma once

#include <extmath.hpp>
#include <entt/entt.hpp>

class SpringMesh;

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
    void setEndpoint(const Vector3f& endpoint);
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

// COMPONENT: Rigid body
//  - controls: PhysicsBody
//  - sibling: DebugColor, BoxCollider
struct RigidBody
{
    static constexpr std::size_t page_size = 65536u;


    Vector3f angMomentum = Vector3f::Zero();
    Matrix3f rot = euler(Vector3f::Zero()).toRotationMatrix();
    // Rest inertial tensor
    Matrix3f J;
    // Inverse of rest inertial tensor
    Matrix3f invJ;
    float elasticity = 0.99f;

    RigidBody(const ColliderBox& collider);
    /**
     * @brief Resolve a collision given necessary values
     * 
     * @param pb The sibling physics body
     * @param penetration Amount of penetration into other object
     * @param cnorm Normal vector of collision (points out from collision point)
     * @param contact Vector from center of mass to contact point
     */
    void collidePoint(PhysicsBody& pb, float penetration, const Vector3f& cnorm, const Vector3f& contact);

    void collide(
        PhysicsBody& pb, RigidBody& rb2, PhysicsBody& pb2,
        float penetration, const Vector3f& cnorm, const Vector3f& r1, const Vector3f& r2);
    
    // Returns current inertial tensor computed from rest intertial tensor and rotation matrix
    Matrix3f inertialTensor() const;
    // Returns current angular velocity computed from angular momentum and inertial tensor
    Vector3f angVel() const;
    // Returns the intersection point for given ray
    std::optional<Vector3f> intersect(const PhysicsBody& pb, const ColliderBox& collider, const Ray& ray) const;
    // Returns vertices
    std::array<Vector3f, 8> vertices(const PhysicsBody& pb, const ColliderBox& collider) const;
    // Returns faces as planes
    std::array<Plane, 6> faces(const PhysicsBody& pb, const ColliderBox& collider) const;
    // Applies impulse at specified point with specified vector
    void applyImpulse(PhysicsBody& pb, const Vector3f &point, const Vector3f &impulse);
};

class ColliderInteriorBox : public AABB
{
public:
    using AABB::AABB;

    bool collide(PhysicsBody& body) const;
    bool collide(RigidBody& rb, PhysicsBody& pb, ColliderBox& collider) const;
    bool collide(SpringMesh& mesh) const;
};

struct Spring
{
    static constexpr std::size_t page_size = 65536u;

    float restLength;
    size_t startIdx;
    size_t endIdx;
};

class SpringMesh
{
private:

    std::unordered_map<size_t, size_t> bdryIdxMap;
public:
    std::vector<Vector3f> surfaceVertices;
    std::vector<size_t> surfaceElems;
    VectorXf particles;
    VectorXf velocities;
    VectorXf gravityF;
    VectorXf impulseF;
    std::vector<Spring> springs;
    float stiffness = 150.0f;
    float damping = 0.1f;
    // Stiffness matrix
    SparseMatrix<float> stiffnessMat;
    // Mass matrix
    SparseMatrix<float> massMat;

    SpringMesh(const std::string& elePath, const std::string& nodePath);

    void simulate(float dt);
    void resetForces();
    void evalForces(const VectorXf& V, const VectorXf& X, VectorXf& F) const;
    void applyImpulse(const Vector3f& point, const Vector3f& impulse);
    std::optional<Vector3f> intersect(const Ray& ray) const;
};

namespace Physics {
    void simulate(float dt, RigidBody& rb, PhysicsBody& b);
    void collide(float dt, entt::registry& reg);
};