#include <algorithm>
#include <iostream>
#include <fstream>
#include <physics.hpp>
#include <model.hpp>
#include <spdlog/spdlog.h>

const Matrix4f DebugRay::transform() const {
    return identityTransform()
        .translate(this->pos)
        .rotate(euler(this->rot))
        .scale(Vector3f(1.0f, 1.0f, std::sqrt(this->length)))
        .matrix();
}

void DebugRay::setEndpoint(const Vector3f &endpoint) {
    this->length = (endpoint - this->pos).norm();
    this->rot = towards(this->pos, endpoint);
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
    const std::array<Vector3f, 6u> facePositions = {
        Vector3f(this->max.x(), 0.0f, 0.0f),
        Vector3f(this->min.x(), 0.0f, 0.0f),
        Vector3f(0.0f, this->max.y(), 0.0f),
        Vector3f(0.0f, this->min.y(), 0.0f),
        Vector3f(0.0f, 0.0f, this->max.z()),
        Vector3f(0.0f, 0.0f, this->min.z())
    };
    // Create transform matrix to translate and rotate collider vertices
    const std::array<Vector3f, 8> verts = rb.vertices(pb, collider);
    for (size_t i = 0; i < 6; i++) {
        Vector3f contact = Vector3f::Zero();
        const Vector3f faceNormal = -AABB::faceNormals[i];
        int contacts = 0;
        for (const Vector3f& v : verts) {
            // Project vertex onto inverted face normal
            const float penetration = -(v - facePositions[i]).dot(faceNormal);
            if (penetration > 0.0f) {
                contact += v;
                contacts++;
            }
        }
        if (contacts > 0) {
            contact /= (float)contacts;
            rb.collidePoint(pb, -(contact - facePositions[i]).dot(faceNormal), -faceNormal, contact - pb.pos);
            return true;
        }
    }
    return false;
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

void RigidBody::collidePoint(PhysicsBody& pb, float penetration, const Vector3f& cnorm, const Vector3f& contact) {
    const Vector3f velIn = pb.vel;
    const Matrix3f invInertialTensor = this->inertialTensor().inverse();

    // Reaction impulse magnitude
    const float impulseMagnitude =
        -(((1.0f + this->elasticity) * pb.vel).dot(cnorm))
        / (1.0f / pb.mass + ((invInertialTensor * contact.cross(cnorm)).cross(contact)).dot(cnorm));
    // Update linear velocity
    pb.vel += ((impulseMagnitude / pb.mass) * cnorm);
    // Update angular momentum
    this->angMomentum += impulseMagnitude * contact.cross(cnorm);

    // Resolve penetration, update position
    pb.pos -= penetration * cnorm;
}

void RigidBody::collide(
    PhysicsBody& pb, RigidBody& rb2, PhysicsBody& pb2,
    float penetration, const Vector3f& cnorm, const Vector3f& r1, const Vector3f& r2)
{
    const float impulseMagnitude = 
        -(((1.0f + this->elasticity) * (pb.vel - pb2.vel)).dot(cnorm))
        / (1.0f / pb.mass + 1.0f / pb2.mass + ((this->invJ * r1.cross(cnorm)).cross(r1) + (rb2.invJ * r2.cross(cnorm)).cross(r2)).dot(cnorm));
    
    pb.vel -= (impulseMagnitude / pb.mass) * cnorm;
    pb2.vel += (impulseMagnitude / pb2.mass) * cnorm;

    this->angMomentum -= impulseMagnitude * r1.cross(cnorm);
    rb2.angMomentum += impulseMagnitude * r2.cross(cnorm);

    pb.pos -= penetration * cnorm;
    pb2.pos += penetration * cnorm;
}

Matrix3f RigidBody::inertialTensor() const {
    return this->rot * this->J * this->rot.transpose();
}

Vector3f RigidBody::angVel() const {
    return this->inertialTensor().inverse() * this->angMomentum;
}

std::optional<Vector3f> RigidBody::intersect(const PhysicsBody& pb, const ColliderBox& collider, const Ray& ray) const {
    const AABB box(-collider.halfExtents, collider.halfExtents);
    const Matrix4f bodyTransform = transform(pb.pos, this->rot);
    const std::optional<Vector3f> intersection = box.intersect(ray.transformed(bodyTransform.inverse()));
    if (intersection) {
        return transformPoint(*intersection, bodyTransform);
    }
    return std::nullopt;
}

std::array<Vector3f, 8> RigidBody::vertices(const PhysicsBody &pb, const ColliderBox &collider) const {
    const Matrix4f bodyTransform = transform(pb.pos, this->rot.eulerAngles(0,1,2));
    std::array<Vector3f, 8> verts = AABB(-collider.halfExtents, collider.halfExtents).vertices();
    for (Vector3f& v : verts) {
        v = transformPoint(v, bodyTransform);
    }
    return verts;
}

std::array<Plane, 6> RigidBody::faces(const PhysicsBody &pb, const ColliderBox &collider) const {
    const std::array<Vector3f, 6u> facePositions = {
        Vector3f(collider.halfExtents.x(), 0.0f, 0.0f),
        Vector3f(-collider.halfExtents.x(), 0.0f, 0.0f),
        Vector3f(0.0f, collider.halfExtents.y(), 0.0f),
        Vector3f(0.0f, -collider.halfExtents.y(), 0.0f),
        Vector3f(0.0f, 0.0f, collider.halfExtents.z()),
        Vector3f(0.0f, 0.0f, -collider.halfExtents.z())
    };
    const Matrix4f bodyTransform = transform(pb.pos, this->rot);
    std::array<Plane, 6> faces;
    for (size_t i = 0; i < 6; i++) {
        faces[i] = {
            transformPoint(facePositions[i], bodyTransform),
            transformDir(AABB::faceNormals[i], bodyTransform)};
    }
    return faces;
}

void RigidBody::applyImpulse(PhysicsBody& pb, const Vector3f &point, const Vector3f &impulse) {
    this->angMomentum += point.cross(impulse);
    pb.vel += impulse / pb.mass;
}

void Physics::simulate(float dt, RigidBody& rb, PhysicsBody& b) {
    Matrix3f deltaRot = skew(rb.rot * rb.invJ * rb.rot.transpose() * rb.angMomentum) * (rb.rot);

    b.acc = {0.0f, -9.81f, 0.0f};
    b.vel += b.mass * b.acc * dt;
    b.pos += b.vel * dt;
    rb.rot = Quaternionf(rb.rot + deltaRot * dt).normalized().toRotationMatrix();
}

void Physics::collide(float dt, entt::registry& reg) {
    auto view = reg.view<RigidBody, PhysicsBody, ColliderBox>();
    for (auto entityA : view) {
        auto [rigidBodyA, physicsBodyA, colliderA] = view.get<RigidBody, PhysicsBody, ColliderBox>(entityA);

        for (auto entityB : view) {
            if (entityA == entityB) continue;
            auto [rigidBodyB, physicsBodyB, colliderB] = view.get<RigidBody, PhysicsBody, ColliderBox>(entityB);

            // Look for intersections between A's vertices and B's faces
            const AABB box(-colliderA.halfExtents, colliderA.halfExtents);
            
            // const std::array<Vector3f, 6u> facePositions = {
            //     Vector3f(colliderB.max.x(), 0.0f, 0.0f),
            //     Vector3f(colliderB.min.x(), 0.0f, 0.0f),
            //     Vector3f(0.0f, colliderB.max.y(), 0.0f),
            //     Vector3f(0.0f, colliderB.min.y(), 0.0f),
            //     Vector3f(0.0f, 0.0f, colliderB.max.z()),
            //     Vector3f(0.0f, 0.0f, colliderB.min.z())
            // };
            
        }
    }
}

SpringMesh::SpringMesh(const std::string& elePath, const std::string& nodePath) {
    // Read node file
    std::ifstream nodeFile(nodePath);
    if (!nodeFile.is_open()) {
        throw std::runtime_error("Failed to open node file");
    }
    int numNodes, dims, attrib, boundaryMarker;
    nodeFile >> numNodes >> dims >> attrib >> boundaryMarker;
    spdlog::debug("reading {} nodes from {} [{},{},{}]", numNodes, nodePath, dims, attrib, boundaryMarker);
    while (nodeFile) {
        int idx, isBoundary;
        float x, y, z;
        nodeFile >> idx >> x >> y >> z >> isBoundary;
        this->vertices.push_back({x, y, z});
        if (isBoundary) {
            this->boundaryVertices.push_back({x, y, z});
        }
    }
    nodeFile.close();

    // Read tetrahedron element file, create springs
    std::ifstream eleFile(elePath);
    if (!eleFile.is_open()) {
        throw std::runtime_error("Failed to open element file");
    }
    int numElems, numNodesPerElem, col3;
    eleFile >> numElems >> numNodesPerElem >> col3;
    spdlog::debug("reading {} elements from {} with {} nodes per element", numElems, elePath, numNodesPerElem);
    while (eleFile) {
        int idx, n1, n2, n3, n4;
        eleFile >> idx >> n1 >> n2 >> n3 >> n4;
        const std::array<std::pair<size_t, size_t>, 6u> edges = {
            std::make_pair((size_t)n1, (size_t)n2),
            std::make_pair((size_t)n1, (size_t)n3),
            std::make_pair((size_t)n1, (size_t)n4),
            std::make_pair((size_t)n2, (size_t)n3),
            std::make_pair((size_t)n2, (size_t)n4),
            std::make_pair((size_t)n3, (size_t)n4)
        };
        for (const std::pair<size_t, size_t>& edge : edges) {
            this->springs.push_back(Spring{
                .restLength=(this->vertices.at(edge.second) - this->vertices.at(edge.first)).norm(),
                .startIdx=edge.first, .endIdx=edge.second});
        }
    }
    spdlog::debug("created {} springs", this->springs.size());
}