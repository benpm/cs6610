#pragma once

#include <glad/glad.h>
#include <extmath.hpp>
#include <cyTriMesh.h>
#include <cyGL.h>

class Model
{
private:
    cyTriMesh mesh;
    // Index of VBO for storing vertex data
    GLuint vertVBO;
public:
    Vector3f pos = {0.0f, 0.0f, 0.0f};
    Vector3f rot = {0.0f, 0.0f, 0.0f};
    Vector3f scale = {1.0f, 1.0f, 1.0f};

    Model(const char* filename, const cyGLSLProgram& prog);

    // Returns transformation matrix for this model's current transform
    const Matrix4f transform() const;
    // Binds my VBO and draws
    void draw(cyGLSLProgram& prog) const;
};