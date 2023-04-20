#pragma once

#include <extmath.hpp>
#include <glad/glad.h>
#include <cyGL.h>

struct BufferObject
{
    GLuint glID = GL_INVALID_INDEX;
    GLenum target;
};

struct ImageObject
{
    GLuint texID = GL_INVALID_INDEX;
    GLenum format;
    GLenum access; // GL_READ_ONLY, GL_WRITE_ONLY, GL_READ_WRITE
};

class ComputeShader
{
private:
    GLuint shaderID = GL_INVALID_INDEX;
    GLuint programID = GL_INVALID_INDEX;
    // Maps binding indices to buffer IDs
    std::unordered_map<GLuint, BufferObject> bufBindIdxMap;
    // Maps image units to texture IDs
    std::unordered_map<GLuint, ImageObject> imgBindIdxMap;
public:
    // Compiles compute shader from file at given path
    void compile(const std::string& path);
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
    void zeroBufferData(GLuint bindingIdx, size_t offset, size_t bytes, GLubyte value = 0u);
    // Sets the data of the buffer associated with the given buffer name to zero
    void zeroBufferData(const char* name, size_t offset, size_t bytes, GLubyte value = 0u);
    // Binds all associated buffers and images
    void bind();
    // Creates a new image object and associates it with the given binding index
    GLuint createImage(GLuint imgUnit, GLenum format, GLenum access, const Vector3i& size, bool initStorage=false);
    // Sets the data of the image associated with the given binding index
    void setImageData(GLuint imgUnit, const void* data, const Vector3i &size, const Vector3i &offset = {0,0,0});
    // Zeros the data of the image associated with the given binding index
    void zeroImageData(GLuint imgUnit, const Vector3i &size, const Vector3i &offset = {0,0,0});
    // Runs compute shader, blocking until complete
    void run(const Vector3i& groups = {1, 1, 1});
    // Returns buffer id from binding index
    GLuint bufId(GLuint bindingIdx);
    // Returns the binding index of the buffer with the given name
    GLuint bufBindingIdx(const char* name, GLenum target = GL_SHADER_STORAGE_BUFFER);
    // Sets a uniform value
    void setUniform(const char* name, GLuint value);
    // Reads a single value from the buffer associated with the given binding index
    template<typename T> T readBufferData(GLuint bindingIdx, size_t offset = 0u, GLenum target = GL_SHADER_STORAGE_BUFFER) {
        glUseProgram(this->programID); $gl_err();
        glBindBuffer(target, this->bufBindIdxMap[bindingIdx].glID); $gl_err();
        T data;
        glGetBufferSubData(target, offset, sizeof(T), &data); $gl_err();
        glBindBuffer(target, 0); $gl_err();
        return data;
    }
    template<typename T> T readBufferData(const char* name, size_t offset = 0u, GLenum target = GL_SHADER_STORAGE_BUFFER) {
        return this->readBufferData<T>(this->bufBindingIdx(name, target), offset, target);
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
    template<typename T> std::vector<T> readBufferDataArray(const char* name, size_t count, size_t offset = 0u, GLenum target = GL_SHADER_STORAGE_BUFFER) {
        return this->readBufferDataArray<T>(this->bufBindingIdx(name, target), count, offset, target);
    }
};