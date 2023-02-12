#include <physics.hpp>

const Matrix4f DebugRay::transform() const {
    return identityTransform()
        .translate(this->pos)
        .rotate(euler(this->rot))
        .scale(Vector3f(1.0f, 1.0f, this->length))
        .matrix();
}
