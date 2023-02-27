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
    const uint32_t texUnitID = this->nextTexUnitID++;

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
    glBindTexture(GL_TEXTURE_2D, 0); $gl_err();

    const std::string name = fs::path(path).stem().string();

    this->idMap.emplace(texUnitID, TextureData {
        .bindID = bindID, .texUnitID = texUnitID, .type = GL_TEXTURE_2D});
    this->map.emplace(name, texUnitID);
    return texUnitID;
}

uint32_t TextureCollection::add(const std::string& name, GLuint bindID) {
    const uint32_t texUnitID = this->nextTexUnitID++;
    this->idMap.emplace(texUnitID, TextureData {
        .bindID = bindID, .texUnitID = texUnitID, .type = GL_TEXTURE_2D});
    this->map.emplace(name, texUnitID);
    return texUnitID;
}

void TextureCollection::bind(cyGLSLProgram& prog) const {
    // Bind textures to texture units
    std::vector<GLint> texUnits2D(16u, 0);
    std::vector<GLint> texUnitsCube(16u, 0);
    for (const auto& [name, texUnitID] : this->map) {
        const TextureData& texData = this->idMap.at(texUnitID);
        glActiveTexture(GL_TEXTURE0 + (GLenum)texData.texUnitID); $gl_err();
        glBindTexture(texData.type, texData.bindID); $gl_err();
        switch (texData.type) {
            case GL_TEXTURE_2D:
                texUnits2D[texData.texUnitID] = texData.texUnitID;
                break;
            case GL_TEXTURE_CUBE_MAP:
                texUnitsCube[texData.texUnitID] = texData.texUnitID;
                break;
            default:
                spdlog::error("Unknown texture type");
                break;
        }
    }
    prog.SetUniform1("uTex", texUnits2D.data(), texUnits2D.size()); $gl_err();
    prog.SetUniform1("uCubeTex", texUnitsCube.data(), texUnitsCube.size()); $gl_err();
}

void TextureCollection::bind(cyGLSLProgram& prog, const std::string& name, const std::string& uniform) const {
    const TextureData& texData = this->get(name);
    glActiveTexture(GL_TEXTURE0 + (GLenum)texData.texUnitID); $gl_err();
    glBindTexture(texData.type, texData.bindID); $gl_err();
    prog.SetUniform(uniform.c_str(), (GLint)texData.texUnitID); $gl_err();
}

constexpr std::array<GLenum, 6> cubeMapFaces = {
    GL_TEXTURE_CUBE_MAP_POSITIVE_X,
    GL_TEXTURE_CUBE_MAP_NEGATIVE_X,
    GL_TEXTURE_CUBE_MAP_POSITIVE_Y,
    GL_TEXTURE_CUBE_MAP_NEGATIVE_Y,
    GL_TEXTURE_CUBE_MAP_POSITIVE_Z,
    GL_TEXTURE_CUBE_MAP_NEGATIVE_Z
};

uint32_t TextureCollection::addCubemap(const std::string& name, const std::string& dirName) {
    const uint32_t texUnitID = this->nextTexUnitID++;

    GLuint bindID;
    glGenTextures(1, &bindID); $gl_err();
    glBindTexture(GL_TEXTURE_CUBE_MAP, bindID); $gl_err();

    assert(fs::exists(dirName));

    const std::array<std::string, 6u> paths {
        fs::path(dirName) / "posx.png",
        fs::path(dirName) / "negx.png",
        fs::path(dirName) / "posy.png",
        fs::path(dirName) / "negy.png",
        fs::path(dirName) / "posz.png",
        fs::path(dirName) / "negz.png"
    };

    for (size_t i = 0; i < 6u; i++) {
        const auto [data, width, height] = getImageData(paths[i]);
        glTexImage2D(cubeMapFaces[i], 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data.data()); $gl_err();
    }

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR); $gl_err();
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR); $gl_err();
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); $gl_err();
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); $gl_err();
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE); $gl_err();
    glGenerateMipmap(GL_TEXTURE_CUBE_MAP); $gl_err();
    
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0); $gl_err();

    this->idMap.emplace(texUnitID, TextureData {
        .bindID = bindID, .texUnitID = texUnitID, .type = GL_TEXTURE_CUBE_MAP});
    this->map.emplace(name, texUnitID);
    return texUnitID;
}

uint32_t TextureCollection::addCubemap(const std::string& name, uint32_t width, uint32_t height) {
    const uint32_t texUnitID = this->nextTexUnitID++;

    GLuint bindID;
    glGenTextures(1, &bindID); $gl_err();
    glBindTexture(GL_TEXTURE_CUBE_MAP, bindID); $gl_err();

    for (size_t i = 0; i < 6u; i++) {
        glTexImage2D(cubeMapFaces[i], 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr); $gl_err();
    }

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR); $gl_err();
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR); $gl_err();
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); $gl_err();
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); $gl_err();
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE); $gl_err();
    
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0); $gl_err();

    this->idMap.emplace(texUnitID, TextureData {
        .bindID = bindID, .texUnitID = texUnitID, .type = GL_TEXTURE_CUBE_MAP});
    this->map.emplace(name, texUnitID);
    return texUnitID;
}

const TextureData& TextureCollection::get(uint32_t texUnitID) const {
    return this->idMap.at(texUnitID);
}

const TextureData& TextureCollection::get(const std::string& name) const {
    return this->get(this->map.at(name));
}