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
    GLuint createBuffer(GLuint bindingIdx, size_t bytes, GLenum target = GL_SHADER_STORAGE_BUFFER);
    void setBufferData(GLuint bindingIdx, const void* data, size_t offset, size_t bytes, GLenum target = GL_SHADER_STORAGE_BUFFER);
    void zeroBufferData(GLuint bindingIdx, size_t offset, size_t bytes, GLenum target = GL_SHADER_STORAGE_BUFFER);
    void run();
    // Returns buffer id from binding index
    GLuint bufId(GLuint bindingIdx);
    template<typename T> T readBufferData(GLuint bindingIdx, size_t offset = 0u, GLenum target = GL_SHADER_STORAGE_BUFFER) {
        glUseProgram(this->programID); $gl_err();
        GLuint bufferID = this->bufBindIdxMap[bindingIdx];
        glBindBuffer(target, bufferID); $gl_err();
        T data;
        glGetBufferSubData(target, offset, sizeof(T), &data); $gl_err();
        glBindBuffer(target, 0); $gl_err();
        return data;
    }
    template<typename T> std::vector<T> readBufferDataArray(GLuint bindingIdx, size_t count, size_t offset = 0u, GLenum target = GL_SHADER_STORAGE_BUFFER) {
        glUseProgram(this->programID); $gl_err();
        GLuint bufferID = this->bufBindIdxMap[bindingIdx];
        glBindBuffer(target, bufferID); $gl_err();
        std::vector<T> data(count);
        glGetBufferSubData(target, offset, sizeof(T) * count, data.data()); $gl_err();
        glBindBuffer(target, 0); $gl_err();
        return data;
    }
};