#pragma once

#include <extmath.hpp>
#include <glad/glad.h>
#include <cyGL.h>

class ComputeShader
{
private:
    GLuint shaderID;
    GLuint programID;
    // Maps SSBO binding indices to generate gl IDs
    std::unordered_map<GLuint, GLuint> bufBindIdxMap;
public:
    void compile(const std::string& path);
    GLuint createBuffer(GLuint bindingIdx, size_t bytes);
    void setBufferData(GLuint bindingIdx, const void* data, size_t offset, size_t bytes);
    void run();
};