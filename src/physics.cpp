#include <algorithm>
#include <iostream>
#include <fstream>
#include <unordered_set>
#include <chrono>
#include <physics.hpp>
#include <model.hpp>
#include <spdlog/spdlog.h>

struct hash_pair {
    template <class T1, class T2>
    size_t operator()(const std::pair<T1, T2>& p) const
    {
        auto hash1 = std::hash<T1>{}(p.first);
        auto hash2 = std::hash<T2>{}(p.second);
 
        if (hash1 != hash2) {
            return cantor(hash1, hash2);             
        }
        return hash1;
    }
};

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

bool ColliderInteriorBox::collide(SpringMesh &mesh) const {
    const std::array<Vector3d, 6u> facePositions = {
        Vector3d(this->max.x(), 0.0f, 0.0f),
        Vector3d(this->min.x(), 0.0f, 0.0f),
        Vector3d(0.0f, this->max.y(), 0.0f),
        Vector3d(0.0f, this->min.y(), 0.0f),
        Vector3d(0.0f, 0.0f, this->max.z()),
        Vector3d(0.0f, 0.0f, this->min.z())
    };
    // Create transform matrix to translate and rotate collider vertices
    for (size_t i = 0; i < 6; i++) {
        const Vector3d faceNormal = -AABB::faceNormals[i].cast<double>();
        for (size_t j = 0; j < mesh.x.size() / 3; j++) {
            Ref<Vector3d> p = mesh.x.segment<3>(j*3);
            Ref<Vector3d> v = mesh.v.segment<3>(j*3);

            // Project vertex onto inverted face normal
            const double penetration = -(p - facePositions[i]).dot(faceNormal);
            if (penetration > 0.0f) {
                p += penetration * faceNormal;
                mesh.impulseF.segment<3>(j*3) += 0.75 * faceNormal * v.norm() * 2.0;
            }
        }
    }
    return true;
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

SpringMesh::SpringMesh(const std::string& elePath, const std::string& nodePath) {
    std::vector<Vector3f> vertices;

    // Read node file
    std::ifstream nodeFile(nodePath);
    if (!nodeFile.is_open()) {
        throw std::runtime_error("Failed to open node file");
    }
    int numNodes, dims, attrib, boundaryMarker;
    nodeFile >> numNodes >> dims >> attrib >> boundaryMarker;
    spdlog::debug("reading {} nodes from {} [{},{},{}]", numNodes, nodePath, dims, attrib, boundaryMarker);
    AABB box;
    while (nodeFile) {
        int idx, isBoundary;
        float x, y, z;
        nodeFile >> idx >> x >> y >> z >> isBoundary;
        Vector3f v(x, y, z);
        vertices.push_back(v);
        if (isBoundary) {
            this->bdryIdxMap.emplace((size_t)(idx - 1), this->surfaceVertices.size());
            this->surfaceVertices.push_back(v);
            for (size_t j = 0; j < 3u; j++) {
                box.min[j] = std::min(box.min[j], v[j]);
                box.max[j] = std::max(box.max[j], v[j]);
            }
        }
        if (vertices.size() == numNodes) break;
    }
    nodeFile.close();

    // Normalize particles
    const Vector3f boundSize = box.max - box.min;
    const float maxBoundSize = std::max(std::max(boundSize.x(), boundSize.y()), boundSize.z());
    for (Vector3f& v : vertices) {
        v = (v - box.center()) / maxBoundSize;
    }
    for (Vector3f& v : this->surfaceVertices) {
        v = (v - box.center()) / maxBoundSize;
    }

    // Read tetrahedron element file, create springs
    std::ifstream eleFile(elePath);
    if (!eleFile.is_open()) {
        throw std::runtime_error("Failed to open element file");
    }
    int numTet, numNodesPerElem, col3;
    eleFile >> numTet >> numNodesPerElem >> col3;
    spdlog::debug("reading {} elements from {} with {} nodes per element", numTet, elePath, numNodesPerElem);
    int tetCount = 0;
    std::unordered_set<std::pair<size_t, size_t>, hash_pair> springPairs;
    while (eleFile) {
        // Read indices and determine if exterior triangle
        int idx, t[4];
        eleFile >> idx;
        std::vector<size_t> triIdxs;
        Vector3f inner = Vector3f::Zero();
        for (int i = 0; i < 4; i++) {
            eleFile >> t[i];
            t[i] -= 1;
            if (this->bdryIdxMap.count(t[i])) {
                triIdxs.push_back(this->bdryIdxMap.at(t[i]));
            } else {
                inner = vertices.at(t[i]);
            }
        }

        // Create surface faces for rendering
        if (triIdxs.size() == 3) {
            // Check if normal of face is pointing towards inner
            const Vector3f& a = this->surfaceVertices.at(triIdxs[0]);
            const Vector3f& b = this->surfaceVertices.at(triIdxs[1]);
            const Vector3f& c = this->surfaceVertices.at(triIdxs[2]);
            const Vector3f normal = (b - a).cross(c - a);
            if (normal.dot(inner - a) > 0.0f) {
                std::swap(triIdxs[1], triIdxs[2]);
            }

            for (int i : triIdxs) {
                this->surfaceElems.push_back(i);
            }
        } else if (triIdxs.size() == 4) {
            for (int i : {0,2,1, 3,0,1, 3,1,2, 2,0,3}) {
                this->surfaceElems.push_back(triIdxs[i]);
            }
        }

        // Insert springs at edges of tetrahedron, skipping already created ones
        const std::array<std::pair<size_t, size_t>, 6u> edges = {
            std::make_pair(t[0], t[1]),
            std::make_pair(t[0], t[2]),
            std::make_pair(t[0], t[3]),
            std::make_pair(t[1], t[2]),
            std::make_pair(t[1], t[3]),
            std::make_pair(t[2], t[3])
        };
        for (const std::pair<size_t, size_t>& edge : edges) {
            if (springPairs.count(edge)) continue;
            if (springPairs.count({edge.second, edge.first})) continue;
            springPairs.insert(edge);
            this->springs.push_back(Spring{
                .restLength=(double)(vertices.at(edge.second) - vertices.at(edge.first)).norm(),
                .startIdx=edge.first, .endIdx=edge.second});
        }

        // Break if we've read all the tetrahedra
        tetCount += 1;
        if (tetCount == numTet) {
            break;
        }
    }

    // Initialize stiffness matrix
    this->K = SparseMatrix<double>(vertices.size() * 3, vertices.size() * 3);
    this->K.reserve(Eigen::VectorXi::Constant(vertices.size() * 3, 6 * 9));
    for (const Spring& spring : this->springs) {
        const size_t i = spring.startIdx;
        const size_t j = spring.endIdx;
        for (int k = 0; k < 3; k++) {
            for (int l = 0; l < 3; l++) {
                this->K.coeffRef(i*3 + k, j*3 + l) = 1.0;
                this->K.coeffRef(j*3 + k, i*3 + l) = 1.0;
                this->K.coeffRef(i*3 + k, i*3 + l) = 1.0;
                this->K.coeffRef(j*3 + k, j*3 + l) = 1.0;
            }
        }
    }
    this->K.makeCompressed();
    spdlog::debug("stiffness matrix has {} non-zero entries", this->K.nonZeros());

    // Generate mass matrix M
    this->M = SparseMatrix<double>(vertices.size() * 3, vertices.size() * 3);
    this->M.reserve(Eigen::VectorXi::Constant(vertices.size() * 3, 1));
    const double mass = 1.0;
    for (size_t i = 0; i < vertices.size(); i++) {
        this->M.coeffRef(i*3 + 0, i*3 + 0) = mass;
        this->M.coeffRef(i*3 + 1, i*3 + 1) = mass;
        this->M.coeffRef(i*3 + 2, i*3 + 2) = mass;
    }
    this->M.makeCompressed();
    this->invMassMat = this->M.cwiseInverse();
    this->invMassMat.makeCompressed();

    // Push vertices to particles, initialize global vectors
    this->x.resize(vertices.size() * 3);
    this->gravityF = VectorXd::Zero(this->x.size());
    this->impulseF = VectorXd::Zero(this->x.size());
    this->v = VectorXd::Zero(this->x.size());
    for (size_t i = 0; i < vertices.size(); i++) {
        this->x.segment<3>(i * 3) = vertices.at(i).cast<double>();
        this->gravityF[i * 3 + 1] = -0.25;
    }
    this->initParticles = this->x;

    spdlog::debug("built tet mesh: {} tets, {} springs, {} triangles, {} boundary vertices",
        numTet, this->springs.size(), this->surfaceElems.size() / 3u, this->surfaceVertices.size());
}

Matrix3d dFdX(const Vector3d& X, const double l0, const double kS, const double kD, const double dt) {
    const Matrix3d I = Matrix3d::Identity();
    return
        // dF/dX for spring forces
        kS * (-I + (l0 / X.norm()) * (I - (X * X.transpose()) / X.squaredNorm())) -
        // dF/dX for damping forces
        kD * ((X * X.transpose()) / X.squaredNorm()) * (I / dt);
}

void SpringMesh::evalForces(const VectorXd& V, const VectorXd& X, VectorXd& F) const {
    F.setZero();
    for (const Spring& s : this->springs) {
        const size_t i = s.startIdx;
        const size_t j = s.endIdx;
        const Vector3d u = X.segment<3>(j * 3) - X.segment<3>(i * 3);
        const Vector3d vi = V.segment<3>(i*3);
        const Vector3d vj = V.segment<3>(j*3);
        const Vector3d springForce =
            (this->stiffness() * (u.norm() - s.restLength)) * u.normalized();
        const Vector3d dampingForce = 
            this->damping * (vj - vi).dot(u.normalized()) * u.normalized();
        F.segment<3>(i*3) += springForce + dampingForce;
        F.segment<3>(j*3) -= springForce + dampingForce;
    }
}

void SpringMesh::applyImpulse(const Vector3d &point, const Vector3d &impulse) {
    for (size_t i = 0; i < this->x.size() / 3; i++) {
        const Vector3d p = this->x.segment<3>(i * 3);
        this->v.segment<3>(i * 3) += impulse * (p - point).norm();
    }
}

std::optional<Vector3f> SpringMesh::intersect(const Ray &ray) const {
    bool hit = false;
    Vector3f nearest;
    float nearestDist;
    for (size_t i = 0; i < this->surfaceElems.size(); i += 3) {
        const std::optional<Vector3f> intersect = rayTriangleIntersect(ray,
            this->surfaceVertices[this->surfaceElems[i + 0] * 3],
            this->surfaceVertices[this->surfaceElems[i + 1] * 3],
            this->surfaceVertices[this->surfaceElems[i + 2] * 3]);
        if (intersect) {
            float dist = (*intersect - ray.origin).norm();
            if (!hit || dist < nearestDist) {
                nearestDist = dist;
                nearest = *intersect;
                hit = true;
            }
        }
    }
    if (hit) {
        return {nearest};
    } else {
        return std::nullopt;
    }
}

void SpringMesh::simulate(float dt) {
    // Compute stiffness matrix
    this->K.coeffs().setZero();
    for (const Spring& spring : this->springs) {
        const size_t i = spring.startIdx;
        const size_t j = spring.endIdx;
        const Vector3d u = this->x.segment<3>(j * 3) - this->x.segment<3>(i * 3);
        const Matrix3d Kii = dFdX(u, spring.restLength, this->stiffness(), this->damping, dt);
        const Matrix3d& Kjj = Kii;
        const Matrix3d Kij = -Kii;
        const Matrix3d& Kji = Kij;
        for (int k = 0; k < 3; k++) {
            for (int l = 0; l < 3; l++) {
                this->K.coeffRef(i*3 + k, j*3 + l) += Kij(k, l);
                this->K.coeffRef(j*3 + k, i*3 + l) += Kji(k, l);
                this->K.coeffRef(i*3 + k, i*3 + l) += Kii(k, l);
                this->K.coeffRef(j*3 + k, j*3 + l) += Kjj(k, l);
            }
        }
    }

    // Implicit Euler: solve system for new velocities
    const VectorXd externalF = this->impulseF + this->gravityF;
    switch (this->solver) {
        case Solver::newton:
            this->solveNewton(dt, externalF);
            break;
        case Solver::conjgrad:
            this->solveConjGrad(dt, externalF);
            break;
    }

    this->x += this->v * dt;
    for (const auto [pIdx, sIdx] : this->bdryIdxMap) {
        this->surfaceVertices.at(sIdx) = this->x.segment<3>(pIdx * 3).cast<float>();
    }
    this->impulseF.setZero();
}

void SpringMesh::solveNewton(float dt, const VectorXd& externalF) {
    // Guess for next velocity, init with current velocity
    VectorXd nvel = this->v;
    // Internal forces evaluated at x + nvel*dt
    VectorXd inFnx = VectorXd::Zero(this->x.size());
    this->evalForces(this->v, this->x, inFnx);
    // Force at current time
    const VectorXd F = externalF + inFnx;

    // Newton-raphson method
    auto tStart = std::chrono::high_resolution_clock::now();
    const auto A = this->M - (dt*dt) * this->K;
    SparseLU<SparseMatrix<double>> solver(A);
    if (solver.info() != Success) {
        spdlog::error("coeff matrix decomposition failed");
    }
    auto tEnd = std::chrono::high_resolution_clock::now();
    spdlog::debug("decomposition took {} ms", std::chrono::duration_cast<std::chrono::milliseconds>(tEnd - tStart).count());
    size_t iter = 0;
    double err = 0.0f;
    do {
        // Attempt iterative solve
        auto tStart = std::chrono::high_resolution_clock::now();
        const VectorXd dv = solver.solve(
            this->M * this->v + dt * F - this->M * nvel);
        auto tEnd = std::chrono::high_resolution_clock::now();

        if (solver.info() != Success) {
            spdlog::warn("solving failed");
            break;
        } else if (nvel.hasNaN()) {
            spdlog::warn("solving produced NaN");
            break;
        } else {
            // Update velocity guess, evaluate forces at new guess
            nvel += dv;
            this->evalForces(nvel, this->x + nvel * dt, inFnx);

            // Compute error
            const auto errMat = (this->M * nvel - this->M * this->v - dt * (inFnx + externalF)).cwiseAbs();
            err = errMat.squaredNorm();

            spdlog::debug("  [{}] {} ms, err = {:.3e}, largest err = {:.3e}, max diff = {:.3e}",
                iter,
                std::chrono::duration_cast<std::chrono::milliseconds>(tEnd - tStart).count(),
                err, errMat.maxCoeff(), dv.cwiseAbs().maxCoeff());
        }

        iter += 1;
    } while (err > 0.1 && iter < 10);
    spdlog::debug("solved in {} newton-raphson steps with err {:.3e}", iter, err);

    // Update velocity
    this->v = nvel;
}

void SpringMesh::solveConjGrad(float dt, const VectorXd& externalF) {
    // Evaluate spring and damping forces at current state
    VectorXd internalF(this->x.size());
    this->evalForces(this->v, this->x, internalF);

    const VectorXd F = externalF + internalF;
    LeastSquaresConjugateGradient<SparseMatrix<double>> solver;
    solver.compute(this->M - (dt*dt) * this->K);
    if (solver.info() != Success) {
        spdlog::warn("decomposition failed");
    }

    // Iterative solve with guess
    auto tStart = std::chrono::high_resolution_clock::now();
    this->v = solver.solveWithGuess(
        this->M * this->v + dt * F, this->v);
    auto tEnd = std::chrono::high_resolution_clock::now();

    if (solver.info() != Success) {
        spdlog::warn("solving failed");
    } else if (this->v.hasNaN()) {
        spdlog::warn("solving produced NaN");
    } else {
        spdlog::debug("solved in {} iters, {} ms, error: {:.4e}",
            solver.iterations(), std::chrono::duration_cast<std::chrono::milliseconds>(tEnd - tStart).count(),
            solver.error());
    }
}

void SpringMesh::reset() {
    this->x = this->initParticles;
    this->v.setZero();
    this->impulseF.setZero();
    this->K.coeffs().setZero();
}

void SpringMesh::resetForces() {
    this->v.fill(0.0f);
}
