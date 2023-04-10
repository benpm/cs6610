#include <iostream>
#include <fstream>
#include <sstream>
#include <compute_shader.hpp>
#include <spdlog/spdlog.h>

void checkShaderCompileErr(GLuint shaderID, const std::string& path) {
    GLint success;
    glGetShaderiv(shaderID, GL_COMPILE_STATUS, &success); $gl_err();
    if (!success) {
        char* infoLog = new char[2048];
        glGetShaderInfoLog(shaderID, 2048, nullptr, infoLog); $gl_err();
        spdlog::error("compilation of {} failed:\n{}", path, infoLog);
        delete [] infoLog;
    }
}

void checkProgramLinkErr(GLuint programID, const std::string& path) {
    GLint success;
    glGetProgramiv(programID, GL_LINK_STATUS, &success); $gl_err();
    if (!success) {
        char* infoLog = new char[2048];
        glGetProgramInfoLog(programID, 2048, nullptr, infoLog); $gl_err();
        spdlog::error("linking of {} failed:\n{}", path, infoLog);
        delete [] infoLog;
    }
}

void ComputeShader::compile(const std::string &path) {
    assert(fs::exists(path) && "ComputeShader::compile: path does not exist");

    // Read shader source
    std::ifstream stream(path);
    std::stringstream buffer;
    buffer << stream.rdbuf();
    stream.close();
    std::string source = buffer.str();

    // Compile shader
    const char *sourcePtr = source.c_str();
    this->shaderID = glCreateShader(GL_COMPUTE_SHADER); $gl_err();
    glShaderSource(this->shaderID, 1, &sourcePtr, nullptr); $gl_err();
    glCompileShader(this->shaderID); $gl_err();
    checkShaderCompileErr(this->shaderID, path);

    this->programID = glCreateProgram(); $gl_err();
    glAttachShader(this->programID, this->shaderID); $gl_err();
    glLinkProgram(this->programID); $gl_err();
    checkProgramLinkErr(this->programID, "compute shader");
}

GLuint ComputeShader::createBuffer(GLuint bindingIdx, size_t bytes, GLenum target) {
    glUseProgram(this->programID); $gl_err();
    GLuint bufferID;
    glCreateBuffers(1, &bufferID); $gl_err();
    glBindBuffer(target, bufferID); $gl_err();
    glBufferData(target, bytes, nullptr, GL_DYNAMIC_DRAW); $gl_err();
    glBindBufferBase(target, bindingIdx, bufferID); $gl_err();
    glBindBuffer(target, 0); $gl_err();
    this->bufBindIdxMap[bindingIdx] = bufferID;
    return bufferID;
}

void ComputeShader::setBufferData(GLuint bindingIdx, const void *data, size_t offset, size_t bytes, GLenum target)
{
    assert(this->bufBindIdxMap.count(bindingIdx) && "ComputeShader::setBufferData: bindingIdx not found");

    glBindBuffer(target, this->bufBindIdxMap[bindingIdx]); $gl_err();
    glBufferSubData(target, offset, bytes, data); $gl_err();
    glBindBuffer(target, 0); $gl_err();
}

void ComputeShader::zeroBufferData(GLuint bindingIdx, size_t offset, size_t bytes, GLenum target)
{
    assert(this->bufBindIdxMap.count(bindingIdx) && "ComputeShader::zeroBufferData: bindingIdx not found");

    glBindBuffer(target, this->bufBindIdxMap[bindingIdx]); $gl_err();
    const std::vector<uint8_t> zeros(bytes, 0);
    glBufferSubData(target, offset, bytes, zeros.data()); $gl_err();
    glBindBuffer(target, 0); $gl_err();
}

void ComputeShader::run()
{
    glUseProgram(this->programID); $gl_err();
    glDispatchCompute(1, 1, 1); $gl_err();
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT); $gl_err();
}

GLuint ComputeShader::bufId(GLuint bindingIdx)
{
    assert(this->bufBindIdxMap.count(bindingIdx) && "ComputeShader::bufId: bindingIdx not found");

    return this->bufBindIdxMap[bindingIdx];
}
