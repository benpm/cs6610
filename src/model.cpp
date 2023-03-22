#include <spdlog/spdlog.h>
#include <model.hpp>
#include <mesh.hpp>
#include <camera.hpp>

Model::Model(const MeshData& meshData) {
    this->pivot = meshData.center;
    this->center = meshData.center;
}

const Matrix4f Model::transform() const {
    return identityTransform()
        .translate(this->pos)
        .rotate(euler(this->rot))
        .scale(this->scale)
        .translate(-this->pivot)
        .matrix();
}

const Vector3f Model::transformedCenter() const {
    return identityTransform()
        .translate(this->pos)
        .rotate(euler(this->rot))
        .scale(this->scale) * this->center;
}
