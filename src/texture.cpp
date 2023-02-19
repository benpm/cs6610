#include <filesystem>
#include <texture.hpp>
#include <glad/glad.h>
#include <spdlog/spdlog.h>

uint32_t TextureCollection::add(const std::string& path) {
    spng_ctx* ctx = spng_ctx_new(0);
    if (!ctx) {
        throw std::runtime_error("Failed to create libspng context");
    }

    assert(std::filesystem::exists(path));

    uint32_t colID = this->nextID++;

    FILE* f = fopen(path.c_str(), "rb");
    spng_set_png_file(ctx, f);
    spng_ihdr header;
    spng_get_ihdr(ctx, &header);

    size_t len;
    spng_decoded_image_size(ctx, SPNG_FMT_RGBA8, &len);
    std::vector<uint8_t> data(len);
    spng_decode_image(ctx, data.data(), data.size(), SPNG_FMT_RGBA8, 0);

    GLuint bindID;
    glGenTextures(1, &bindID); $gl_err();
    glBindTexture(GL_TEXTURE_2D, bindID); $gl_err();
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, header.width, header.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data.data()); $gl_err();
    glGenerateMipmap(GL_TEXTURE_2D); $gl_err();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR); $gl_err();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); $gl_err();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT); $gl_err();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT); $gl_err();

    const std::string name = std::filesystem::path(path).stem().string();

    this->map.emplace(name, TextureData {.bindID = bindID, .colID = colID});
    spdlog::trace("tex {} bindID={} colID={}", name, bindID, colID);
    spng_ctx_free(ctx);
    return colID;
}

void TextureCollection::bind(cyGLSLProgram& prog) const {
    // Bind textures to texture units
    std::vector<GLint> colIDs(this->map.size());
    for (const auto& [name, texData] : this->map) {
        glActiveTexture(GL_TEXTURE0 + (GLenum)texData.colID); $gl_err();
        glBindTexture(GL_TEXTURE_2D, texData.bindID); $gl_err();
        colIDs[texData.colID] = (GLint)texData.colID;
    }
    prog.SetUniform1("uTex", colIDs.data(), colIDs.size()); $gl_err();
}