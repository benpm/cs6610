#include <physics.hpp>

const Matrix4f DebugRay::transform() const {
    return identityTransform()
        .translate(this->pos)
        .rotate(euler(this->rot))
        .scale(Vector3f(this->length, this->length, this->length))
        .matrix();
}
