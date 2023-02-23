#include <filesystem>
#include <texture.hpp>
#include <glad/glad.h>
#include <spdlog/spdlog.h>

const std::tuple<std::vector<uint8_t>, uint32_t, uint32_t> getImageData(const std::string& path) {
    assert(fs::exists(path));

    spng_ctx* ctx = spng_ctx_new(0);
    if (!ctx) {
        spdlog::error("Failed to create libspng context");
    }

    FILE* f = fopen(path.c_str(), "rb");
    spng_set_png_file(ctx, f);
    spng_ihdr header;
    spng_get_ihdr(ctx, &header);

    size_t len;
    spng_decoded_image_size(ctx, SPNG_FMT_RGBA8, &len);
    std::vector<uint8_t> data(len);
    spng_decode_image(ctx, data.data(), data.size(), SPNG_FMT_RGBA8, 0);
    spng_ctx_free(ctx);

    return {std::move(data), header.width, header.height};
}

uint32_t TextureCollection::add(const std::string& path) {
    uint32_t colID = this->samplerIDMap["uTex"]++;

    const auto [data, width, height] = getImageData(path);

    GLuint bindID;
    glGenTextures(1, &bindID); $gl_err();
    glBindTexture(GL_TEXTURE_2D, bindID); $gl_err();
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data.data()); $gl_err();
    glGenerateMipmap(GL_TEXTURE_2D); $gl_err();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR); $gl_err();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); $gl_err();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT); $gl_err();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT); $gl_err();

    const std::string name = fs::path(path).stem().string();

    this->map.emplace(name, TextureData {
        .bindID = bindID, .colID = colID, .type = GL_TEXTURE_2D});
    return colID;
}

uint32_t TextureCollection::add(const std::string& name, GLuint bindID) {
    const uint32_t colID = this->samplerIDMap["uTex"]++;
    this->map.emplace(name, TextureData {
        .bindID = bindID, .colID = colID, .type = GL_TEXTURE_2D});
    return colID;
}

void TextureCollection::bind(cyGLSLProgram& prog) const {
    // Bind textures to texture units
    for (const auto [samplerUniform, usedIDs] : this->samplerIDMap) {
        std::vector<GLint> texUnits(usedIDs, 0u);
        for (const auto& [name, texData] : this->map) {
            glActiveTexture(GL_TEXTURE0 + (GLenum)texData.colID); $gl_err();
            glBindTexture(texData.type, texData.bindID); $gl_err();
        }
        prog.SetUniform1(samplerUniform.c_str(), texUnits.data(), usedIDs); $gl_err();
    }
}

constexpr std::array<GLenum, 6> cubeMapFaces = {
    GL_TEXTURE_CUBE_MAP_POSITIVE_X,
    GL_TEXTURE_CUBE_MAP_NEGATIVE_X,
    GL_TEXTURE_CUBE_MAP_POSITIVE_Y,
    GL_TEXTURE_CUBE_MAP_NEGATIVE_Y,
    GL_TEXTURE_CUBE_MAP_POSITIVE_Z,
    GL_TEXTURE_CUBE_MAP_NEGATIVE_Z
};

uint32_t TextureCollection::addCubemap(const std::string& name, const std::array<std::string, 6u>& paths) {
    uint32_t colID = this->samplerIDMap["uCubeTex"]++;

    GLuint bindID;
    glGenTextures(1, &bindID); $gl_err();
    glBindTexture(GL_TEXTURE_CUBE_MAP, bindID); $gl_err();

    for (size_t i = 0; i < 6u; i++) {
        const auto [data, width, height] = getImageData(paths[i]);
        glTexImage2D(cubeMapFaces[i], 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data.data()); $gl_err();
    }

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR); $gl_err();
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR); $gl_err();
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); $gl_err();
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); $gl_err();
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE); $gl_err();

    this->map.emplace(name, TextureData {
        .bindID = bindID, .colID = colID, .type = GL_TEXTURE_CUBE_MAP});
    return colID;
}