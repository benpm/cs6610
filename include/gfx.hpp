#pragma once

#include <tuple>
#include <extmath.hpp>
#include <glad/glad.h>

namespace gfx {
    struct TextureConfig {
        // GL_TEXTURE_2D, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_ARRAY, GL_TEXTURE_2D_ARRAY
        GLenum target = GL_TEXTURE_2D;
        // GL_RGBA, GL_RGB, GL_DEPTH_COMPONENT
        GLenum format = GL_RGBA;
        uint32_t width;
        uint32_t height;
        // GL_CLAMP_TO_EDGE, GL_REPEAT, GL_MIRRORED_REPEAT
        GLenum wrap = GL_REPEAT;
        // GL_UNSIGNED_BYTE, GL_FLOAT
        GLenum storageType = GL_UNSIGNED_BYTE;
        bool mipmap = false;
        bool shadow = false;
        // GL_NEAREST, GL_LINEAR
        GLenum filter = GL_LINEAR;
        // Number of layers when specifying a 3D texture
        GLsizei layers = 1;
        const void* data = nullptr;
        const void* dataCube[6] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    };

    // Generates an opengl texture from a TextureConfig
    GLuint texture(const TextureConfig& conf);
    // Modifies an existing opengl texture from a TextureConfig
    // TODO: This should be smarter about maintaining existing state and config
    void modifyTex(GLuint id, const TextureConfig& conf);

    constexpr std::array<GLenum, 6> cubeMapFaces = {
        GL_TEXTURE_CUBE_MAP_POSITIVE_X,
        GL_TEXTURE_CUBE_MAP_NEGATIVE_X,
        GL_TEXTURE_CUBE_MAP_POSITIVE_Y,
        GL_TEXTURE_CUBE_MAP_NEGATIVE_Y,
        GL_TEXTURE_CUBE_MAP_POSITIVE_Z,
        GL_TEXTURE_CUBE_MAP_NEGATIVE_Z
    };

    const std::array<Vector3f, 6> cubeMapCameraRotations = {
        Vector3f(0.0f, tau4 * 3.0f, tau2), // Facing positive X
        Vector3f(0.0f, tau4 * 1.0f, tau2), // Facing negative X
        Vector3f(tau4 * 3.0f, 0.0f, 0.0f), // Facing positive Y
        Vector3f(tau4 * 1.0f, 0.0f, 0.0f), // Facing negative Y
        Vector3f(0.0f, tau2,        tau2), // Facing positive Z
        Vector3f(0.0f, 0.0f,        tau2), // Facing negative Z
    };
};