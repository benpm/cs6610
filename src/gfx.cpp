#include <spdlog/spdlog.h>
#include <gfx.hpp>

namespace gfx {
    GLuint texture(const TextureConfig& conf) {
        const uint32_t dims = conf.target == GL_TEXTURE_2D ? 2 : 3;
        GLenum internalFormat = conf.format;
        if (conf.format == GL_RGBA && conf.storageType == GL_UNSIGNED_BYTE) {
            internalFormat = GL_RGBA8;
        } else if (conf.format == GL_RGB && conf.storageType == GL_UNSIGNED_BYTE) {
            internalFormat = GL_RGB8;
        } else if (conf.format == GL_DEPTH_COMPONENT && conf.storageType == GL_FLOAT) {
            internalFormat = GL_DEPTH_COMPONENT32F;
        } else if (conf.format == GL_DEPTH_COMPONENT && conf.storageType == GL_UNSIGNED_BYTE) {
            internalFormat = GL_DEPTH_COMPONENT24;
        } else {
            spdlog::error("Unhandled texture format / storage type combination");
        }

        GLuint id;
        glGenTextures(1, &id); $gl_err();
        glBindTexture(conf.target, id); $gl_err();
        switch (conf.target) {
            case GL_TEXTURE_2D:
                glTexImage2D(conf.target, 0, internalFormat, conf.width, conf.height,
                    0, conf.format, conf.storageType, conf.data); $gl_err();
                break;
            case GL_TEXTURE_CUBE_MAP:
                for (uint32_t i = 0; i < 6; ++i) {
                    glTexImage2D(cubeMapFaces[i], 0, internalFormat, conf.width, conf.height,
                        0, conf.format, conf.storageType, conf.dataCube[i]); $gl_err();
                }
                if (conf.format == GL_DEPTH_COMPONENT) {
                    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE); $gl_err();
                    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL); $gl_err();
                }
                break;
            default:
                spdlog::error("Unhandled texture target: {}", conf.target);
                break;
        }
        if (conf.mipmap) {
            glTexParameteri(conf.target, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR); $gl_err();
        } else {
            glTexParameteri(conf.target, GL_TEXTURE_MIN_FILTER, GL_LINEAR); $gl_err();
        }
        glTexParameteri(conf.target, GL_TEXTURE_MAG_FILTER, conf.filter); $gl_err();
        glTexParameteri(conf.target, GL_TEXTURE_WRAP_S, conf.wrap); $gl_err();
        glTexParameteri(conf.target, GL_TEXTURE_WRAP_T, conf.wrap); $gl_err();
        if (dims == 3) {
            glTexParameteri(conf.target, GL_TEXTURE_WRAP_R, conf.wrap); $gl_err();
        }

        // Generate mipmaps if data was provided
        if (conf.mipmap && (conf.data != nullptr || conf.dataCube[0] != nullptr)) {
            glGenerateMipmap(conf.target); $gl_err();
        }

        glBindTexture(conf.target, 0); $gl_err();
        return id;
    }
};