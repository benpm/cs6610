#pragma once

#include <glad/glad.h>
#include <extmath.hpp>
#include <cyTriMesh.h>
#include <cyGL.h>

class Camera;

class Model
{
public:
    Vector3f pos = {0.0f, 0.0f, 0.0f};
    Vector3f rot = {0.0f, 0.0f, 0.0f};
    Vector3f scale = {1.0f, 1.0f, 1.0f};
    Vector3f pivot = {0.0f, 0.0f, 0.0f};

    // Returns transformation matrix for this model's current transform
    const Matrix4f transform() const;
};

// Component for storing dynamic model transforms
struct ModelTransform
{
    static constexpr auto in_place_delete = true;
    Matrix4f transform;
};