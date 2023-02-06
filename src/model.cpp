#include <spdlog/spdlog.h>
#include <model.hpp>
#include <camera.hpp>

Model::Model(const char* filename, const uMaterial& mat) {
    mesh.LoadFromFileObj(filename);
    mesh.ComputeBoundingBox();
    mesh.ComputeNormals();
    this->pivot = toEigen(mesh.GetBoundMax() + mesh.GetBoundMin()) / 2.0f;
    this->mat = mat;
}

const Matrix4f Model::transform() const {
    return identityTransform()
        .translate(this->pos)
        .rotate(euler(this->rot))
        .scale(this->scale)
        .translate(-this->pivot)
        .matrix();
}

void Model::normalize() {
    const Vector3f boundMin = toEigen(mesh.GetBoundMin());
    const Vector3f boundMax = toEigen(mesh.GetBoundMax());
    const Vector3f boundSize = boundMax - boundMin;
    const float maxBoundSize = std::max(std::max(boundSize.x(), boundSize.y()), boundSize.z());
    const float scale = 1.0f / maxBoundSize;
    for (size_t i = 0; i < mesh.NV(); i++) {
        mesh.V(i) *= scale;
    }
    this->pivot *= scale;
}

void Model::addToWorld(
    std::vector<Vector3f>& arrVerts,
    std::vector<uint32_t>& arrElems,
    std::vector<GLsizei>& vCounts,
    std::vector<size_t>& vOffsets,
    std::vector<Matrix4f>& mTransforms,
    std::vector<uMaterial>& mMaterials) const
{
    const size_t vertOffset = arrVerts.size();
    const size_t triOffset = arrElems.size();
    const size_t nElems = mesh.NF() * 3;

    // Add vertex data
    arrVerts.resize(vertOffset + mesh.NV() * 3);
    for (size_t i = 0; i < mesh.NV(); i++) {
        arrVerts[vertOffset + i*3 + 0] = toEigen(mesh.V(i));
        arrVerts[vertOffset + i*3 + 1] = toEigen(mesh.VN(i));
        arrVerts[vertOffset + i*3 + 2] = toEigen(mesh.VN(i));
    }

    // Add triangles
    arrElems.resize(triOffset + nElems);
    for (size_t i = 0; i < mesh.NF(); i++) {
        arrElems[triOffset + i*3 + 0] = mesh.F(i).v[0] + (vertOffset / nVertAttribs);
        arrElems[triOffset + i*3 + 1] = mesh.F(i).v[1] + (vertOffset / nVertAttribs);
        arrElems[triOffset + i*3 + 2] = mesh.F(i).v[2] + (vertOffset / nVertAttribs);
    }

    vCounts.push_back(nElems);
    vOffsets.push_back(triOffset * sizeof(uint32_t));
    mTransforms.push_back(this->transform());
    mMaterials.push_back(this->mat);
}
