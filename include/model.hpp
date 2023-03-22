#pragma once

#include <glad/glad.h>
#undef near
#undef far
#include <extmath.hpp>
#include <cyTriMesh.h>
#include <cyGL.h>

class CameraControl;
struct MeshData;

struct ReflectionProbe {
    static constexpr std::size_t page_size = 65536u;
    // Determines the layer in the cube map array that this probe will use
    //  -1 means none assigned
    int layer = -1;
};

class Model
{
public:
    Vector3f pos = {0.0f, 0.0f, 0.0f};
    Vector3f rot = {0.0f, 0.0f, 0.0f};
    Vector3f scale = {1.0f, 1.0f, 1.0f};
    Vector3f pivot = {0.0f, 0.0f, 0.0f};
    Vector3f center = {0.0f, 0.0f, 0.0f};

    Model() = default;
    Model(const MeshData& meshData);
    // Returns transformation matrix for this model's current transform
    const Matrix4f transform() const;
    // Returns the center position of this model using the current transform
    const Vector3f transformedCenter() const;
};

// Component for storing dynamic model transforms
struct ModelTransform
{
    static constexpr std::size_t page_size = 65536u;
    Matrix4f transform;
};