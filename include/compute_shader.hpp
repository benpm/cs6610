#pragma once

#include <extmath.hpp>
#include <glad/glad.h>
#include <cyGL.h>

struct BufferObject
{
    GLuint glID = GL_INVALID_INDEX;
    GLenum target;
    size_t bytes = 0u;
};

class ComputeShader
{
private:
    GLuint shaderID = GL_INVALID_INDEX;
    GLuint programID = GL_INVALID_INDEX;
    // Maps binding indices to buffer IDs
    std::unordered_map<GLuint, BufferObject> bufBindIdxMap;
    fs::path path;
public:
    // Compiles compute shader from file at given path
    void compile(const std::string& path);
    // Recompiles compute shader from last path given
    void recompile();
    // Creates a new buffer object and associates it with the given binding index
    GLuint createBuffer(GLuint bindingIdx, size_t bytes, GLenum target = GL_SHADER_STORAGE_BUFFER);
    // Creates a new SSBO and associates it with the given named binding
    GLuint createBuffer(const char* name, size_t bytes, GLenum target = GL_SHADER_STORAGE_BUFFER);
    // Associates an existing buffer object with the given binding index
    void assocBuffer(GLuint bindingIdx, GLuint bufferID, GLenum target = GL_SHADER_STORAGE_BUFFER);
    // Associates an existing buffer object with given SSBO binding name
    void assocBuffer(const char* name, GLuint bufferID, GLenum target = GL_SHADER_STORAGE_BUFFER);
    // Binds the buffer associated with the given binding index to given target
    void bindAs(GLuint bindingIdx, GLenum target);
    // Sets the data of the buffer associated with the given binding index
    void setBufferData(GLuint bindingIdx, const void* data, size_t offset, size_t bytes);
    // Sets all data of the buffer associated with the given binding index to zero
    template <typename T> void clearBufferData(GLuint bindingIdx, T value = {}) {
        spdlog::assrt(this->bufBindIdxMap.count(bindingIdx),
            "ComputeShader::zeroBufferData: bindingIdx not found");
        const BufferObject& buf = this->bufBindIdxMap[bindingIdx];

        glBindBuffer(buf.target, buf.glID); $gl_err();
        const std::vector<T> values(buf.bytes / sizeof(T), value);
        glBufferSubData(buf.target, 0u, buf.bytes, values.data()); $gl_err();
        glBindBuffer(buf.target, 0); $gl_err();
    }
    // Binds all associated buffers
    void bindBuffers();
    // Bind shader program
    void bind();
    // Runs compute shader, blocking until complete
    void run(const Vector3i& groups = {1, 1, 1});
    // Returns buffer id from binding index
    GLuint bufId(GLuint bindingIdx);
    // Returns the binding index of the buffer with the given name
    GLuint bufBindingIdx(const char* name, GLenum target = GL_SHADER_STORAGE_BUFFER);
    // Sets a uniform value
    void setUniform(const char* name, GLuint value);
    void setUniform(const char* name, GLfloat value);
    // Reads a single value from the buffer associated with the given binding index
    template<typename T> T readBufferData(GLuint bindingIdx, size_t offset = 0u, GLenum target = GL_SHADER_STORAGE_BUFFER) {
        glUseProgram(this->programID); $gl_err();
        glBindBuffer(target, this->bufBindIdxMap[bindingIdx].glID); $gl_err();
        T data;
        glGetBufferSubData(target, offset, sizeof(T), &data); $gl_err();
        glBindBuffer(target, 0); $gl_err();
        return data;
    }
    // Reads an array of values from the buffer associated with the given binding index
    template<typename T> std::vector<T> readBufferDataArray(GLuint bindingIdx, size_t count, size_t offset = 0u, GLenum target = GL_SHADER_STORAGE_BUFFER) {
        glUseProgram(this->programID); $gl_err();
        glBindBuffer(target, this->bufBindIdxMap[bindingIdx].glID); $gl_err();
        std::vector<T> data(count);
        glGetBufferSubData(target, offset, sizeof(T) * count, data.data()); $gl_err();
        glBindBuffer(target, 0); $gl_err();
        return data;
    }
};