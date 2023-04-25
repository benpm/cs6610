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

    double restLength;
    size_t startIdx;
    size_t endIdx;
};

class SpringMesh
{
private:
    void solveNewton(float dt, const VectorXd& externalF);
    void solveConjGrad(float dt, const VectorXd& externalF);

    std::unordered_map<size_t, size_t> bdryIdxMap;
    VectorXd initx;
public:
    std::vector<Vector3f> surfaceVertices;
    std::vector<size_t> surfaceElems;
    VectorXd x;
    VectorXd v;
    VectorXd gravityF;
    VectorXd impulseF;
    std::unordered_set<size_t> fixed;
    std::vector<Spring> springs;
    float stiffnessFactor = 0.908f;
    float damping = 0.01f;
    bool fixedParticles = false;
    // Stiffness matrix (dF/dx)
    SparseMatrix<double> K;
    // Mass matrix
    SparseMatrix<double> M;

    enum class Solver {
        newton,
        conjgrad
    } solver = Solver::conjgrad;

    SpringMesh(const std::string& elePath, const std::string& nodePath);

    inline float stiffness() const {
        switch (this->solver) {
            case Solver::newton:
                return this->stiffnessFactor * 1000.0f;
            case Solver::conjgrad:
                return this->stiffnessFactor * 1000.0f;
        }
    }
    void simulate(float dt);
    void updateSurfaceMesh();
    void resetForces();
    void evalForces(const VectorXd& V, const VectorXd& X, VectorXd& F) const;
    void applyImpulse(const Vector3d& point, const Vector3d& impulse);
    std::optional<RayHit> intersect(const Ray& ray) const;
    void reset();
};

namespace Physics {
    void simulate(float dt, RigidBody& rb, PhysicsBody& b);
};