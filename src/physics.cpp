#include <physics.hpp>

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
