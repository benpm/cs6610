#include <physics.hpp>

const Matrix4f DebugRay::transform() const {
    return identityTransform()
        .translate(this->pos)
        .rotate(euler(this->rot))
        .scale(Vector3f(std::sqrt(this->length), 1.0f, 1.0f))
        .matrix();
}
