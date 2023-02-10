#include <spdlog/spdlog.h>
#include <model.hpp>
#include <camera.hpp>

const Matrix4f Model::transform() const {
    return identityTransform()
        .translate(this->pos)
        .rotate(euler(this->rot))
        .scale(this->scale)
        .translate(-this->pivot)
        .matrix();
}

