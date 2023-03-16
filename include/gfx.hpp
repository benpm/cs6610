#pragma once

#include <tuple>
#include <extmath.hpp>
#include <glad/glad.h>

namespace gfx {
    struct TextureConfig {
        // GL_TEXTURE_2D, GL_TEXTURE_CUBE_MAP
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
        const void* data = nullptr;
        const void* dataCube[6] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    };

    // Generates an opengl texture from a TextureConfig
    GLuint texture(const TextureConfig& conf);

    constexpr std::array<GLenum, 6> cubeMapFaces = {
        GL_TEXTURE_CUBE_MAP_POSITIVE_X,
        GL_TEXTURE_CUBE_MAP_NEGATIVE_X,
        GL_TEXTURE_CUBE_MAP_POSITIVE_Y,
        GL_TEXTURE_CUBE_MAP_NEGATIVE_Y,
        GL_TEXTURE_CUBE_MAP_POSITIVE_Z,
        GL_TEXTURE_CUBE_MAP_NEGATIVE_Z
    };
};