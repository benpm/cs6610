#pragma once

#include <glad/glad.h>
#include <extmath.hpp>
#include <cyTriMesh.h>
#include <cyGL.h>

class Camera;

class Model
{
private:
    cyTriMesh mesh;
public:
    uMaterial mat;
    Vector3f pos = {0.0f, 0.0f, 0.0f};
    Vector3f rot = {0.0f, 0.0f, 0.0f};
    Vector3f scale = {1.0f, 1.0f, 1.0f};
    Vector3f pivot = {0.0f, 0.0f, 0.0f};

    Model(const char* filename, const uMaterial& mat = {});

    // Returns transformation matrix for this model's current transform
    const Matrix4f transform() const;
    // Normalizes vertex positions
    void normalize();
    // Adds my data to global buffer (static)
    void addToWorld(
        std::vector<Vector3f>& arrVerts,
        std::vector<uint32_t>& arrElems,
        std::vector<GLsizei>& vCounts,
        std::vector<size_t>& vOffsets,
        std::vector<Matrix4f>& mTransforms,
        std::vector<uMaterial>& mMaterials) const;
};