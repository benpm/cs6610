#include <physics.hpp>
#include <model.hpp>
#include <spdlog/spdlog.h>

const Matrix4f DebugRay::transform() const {
    return identityTransform()
        .translate(this->pos)
        .rotate(euler(this->rot))
        .scale(Vector3f(std::sqrt(this->length), 1.0f, 1.0f))
        .matrix();
}

bool ColliderInteriorBox::collide(PhysicsBody &body) const {
    bool collided = false;
    for (size_t i = 0; i < 3; i++) {
        if (body.pos[i] < this->min[i]) {
            body.pos[i] = this->min[i];
            body.vel[i] = -body.vel[i];
            collided = true;
        } else if (body.pos[i] > this->max[i]) {
            body.pos[i] = this->max[i];
            body.vel[i] = -body.vel[i];
            collided = true;
        }
    }
    return collided;
}

bool ColliderInteriorBox::collide(RigidBody& rb, PhysicsBody& pb, ColliderBox& collider) const {
    const std::array<Vector3f, 6u> faceNormals = {
        Vector3f(-1.0f,  0.0f,  0.0f),
        Vector3f( 1.0f,  0.0f,  0.0f),
        Vector3f( 0.0f, -1.0f,  0.0f),
        Vector3f( 0.0f,  1.0f,  0.0f),
        Vector3f( 0.0f,  0.0f, -1.0f),
        Vector3f( 0.0f,  0.0f,  1.0f)
    };
    const std::array<Vector3f, 6u> facePositions = {
        Vector3f(this->max.x(), 0.0f, 0.0f),
        Vector3f(this->min.x(), 0.0f, 0.0f),
        Vector3f(0.0f, this->max.y(), 0.0f),
        Vector3f(0.0f, this->min.y(), 0.0f),
        Vector3f(0.0f, 0.0f, this->max.z()),
        Vector3f(0.0f, 0.0f, this->min.z())
    };
    bool collided = false;
    const Matrix4f m = (Model {
        .pos = pb.pos,
        .rot = rb.rot.eulerAngles(0,1,2)
    }).transform();
    std::array<Vector3f, 8> verts = AABB(-collider.halfExtents, collider.halfExtents).vertices();
    for (Vector3f& v : verts) {
        v = (m * vec4(v)).head<3>();
        for (size_t i = 0; i < 6; i++) {
            // Project vertex onto inverted face normal
            const float penetration = -(v - facePositions[i]).dot(faceNormals[i]);
            if (penetration > 0.0f) {
                rb.collidePoint(pb, penetration, faceNormals[i]);
                collided = true;
            }
        }
    }
    return collided;
}

RigidBody::RigidBody(const ColliderBox& collider) {
    // Calculate rest inertial tensor
    const float x = collider.halfExtents.x();
    const float y = collider.halfExtents.y();
    const float z = collider.halfExtents.z();
    this->J <<
        (y*y + z*z) / 12.0f, 0.0f, 0.0f,
        0.0f, (x*x + z*z) / 12.0f, 0.0f,
        0.0f, 0.0f, (x*x + y*y) / 12.0f;
    this->invJ = this->J.inverse();
}

void RigidBody::collidePoint(PhysicsBody& pb, float penetration, const Vector3f& cnorm) {
    // Impulse vector
    const Vector3f j = (-(1.0f + this->elasticity) * (pb.vel.dot(cnorm)) / (pb.mass + cnorm.dot(this->invJ * cnorm))) * cnorm;
    // Update linear velocity
    pb.vel += j / pb.mass;
    // Update angular momentum
    const Vector3f r = penetration * -cnorm;
    const Vector3f angVel = (this->invJ * this->angMomentum) + (this->invJ * (r.cross(j)));
    this->angMomentum = this->J * angVel;

    // Resolve penetration, update position
    pb.pos += (penetration / 2.0f) * cnorm;
}

void Physics::simulate(float dt, RigidBody& rb, PhysicsBody& b) {
    Matrix3f deltaRot = skew(rb.rot * rb.invJ * rb.rot.transpose() * rb.angMomentum) * (rb.rot);

    b.vel += b.mass * b.acc * dt;
    b.pos += b.vel * dt;
    rb.rot += deltaRot * dt;
    rb.rot = Quaternionf(rb.rot).normalized().toRotationMatrix();
}

void Physics::collide(float dt, entt::registry& reg) {
    auto view = reg.view<RigidBody, PhysicsBody, ColliderBox>();
    for (auto entityA : view) {
        auto [rigidBodyA, physicsBodyA, colliderA] = view.get<RigidBody, PhysicsBody, ColliderBox>(entityA);

        for (auto entityB : view) {
            if (entityA == entityB) continue;
            auto [rigidBodyB, physicsBodyB, colliderB] = view.get<RigidBody, PhysicsBody, ColliderBox>(entityB);

            // Determine if the rotated 3d boxes are intersecting

        }
    }
}