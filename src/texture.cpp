#include <filesystem>
#include <texture.hpp>
#include <glad/glad.h>
#include <spdlog/spdlog.h>
#include <gfx.hpp>

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

    const GLuint bindID = gfx::texture(gfx::TextureConfig{
        .target = GL_TEXTURE_2D,
        .format = GL_RGBA,
        .width = width,
        .height = height,
        .wrap = GL_REPEAT,
        .mipmap = true,
        .data = data.data(),
    });

    const std::string name = fs::path(path).stem().string();

    this->idMap.emplace(texUnitID, TextureData {
        .bindID = bindID, .texUnitID = texUnitID, .type = GL_TEXTURE_2D});
    this->map.emplace(name, texUnitID);
    return texUnitID;
}

uint32_t TextureCollection::add(const std::string& name, GLuint bindID, GLenum type, TextureSampler sampler) {
    const uint32_t texUnitID = this->nextTexUnitID++;
    this->idMap.emplace(texUnitID, TextureData {
        .bindID = bindID, .texUnitID = texUnitID, .type = type, .sampler = sampler});
    this->map.emplace(name, texUnitID);
    return texUnitID;
}

void TextureCollection::bind(cyGLSLProgram& prog) const {
    // Bind textures to texture units
    std::vector<GLint> texUnits2D(16u, 0);
    for (const auto& [name, texUnitID] : this->map) {
        const TextureData& texData = this->idMap.at(texUnitID);
        if (texData.sampler == TextureSampler::normal) {
            switch (texData.type) {
                case GL_TEXTURE_2D:
                    glActiveTexture(GL_TEXTURE0 + (GLenum)texData.texUnitID); $gl_err();
                    glBindTexture(texData.type, texData.bindID); $gl_err();
                    texUnits2D[texData.texUnitID] = texData.texUnitID;
                    break;
                default:
                    spdlog::error("Unknown texture type for normal sampler");
                    break;
            }
        }
    }
    prog.SetUniform1("uTex", texUnits2D.data(), texUnits2D.size()); $gl_err();
}

void TextureCollection::bind(cyGLSLProgram& prog, const std::string& name, const std::string& uniform) const {
    const TextureData& texData = this->get(name);
    glActiveTexture(GL_TEXTURE0 + (GLenum)texData.texUnitID); $gl_err();
    glBindTexture(texData.type, texData.bindID); $gl_err();
    prog.SetUniform(uniform.c_str(), (GLint)texData.texUnitID); $gl_err();
}

uint32_t TextureCollection::addCubemap(const std::string& name, const std::string& dirName) {
    const uint32_t texUnitID = this->nextTexUnitID++;

    assert(fs::exists(dirName));

    const std::array<std::string, 6u> paths {
        fs::path(dirName) / "posx.png",
        fs::path(dirName) / "negx.png",
        fs::path(dirName) / "posy.png",
        fs::path(dirName) / "negy.png",
        fs::path(dirName) / "posz.png",
        fs::path(dirName) / "negz.png"
    };

    uint32_t width, height;
    std::array<std::vector<uint8_t>, 6> faceData;
    for (size_t i = 0; i < 6u; i++) {
        auto [data, w, h] = getImageData(paths[i]);
        if (i == 0) {
            width = w;
            height = h;
        } else {
            assert(width == w && height == h);
        }
        faceData[i] = std::move(data);
    }
    gfx::TextureConfig conf {
        .target = GL_TEXTURE_CUBE_MAP,
        .format = GL_RGBA,
        .width = width,
        .height = height,
        .wrap = GL_CLAMP_TO_EDGE,
        .mipmap = true,
    };
    for (size_t i = 0; i < 6u; i++) {
        conf.dataCube[i] = faceData[i].data();
    }

    const GLuint bindID = gfx::texture(conf);

    this->idMap.emplace(texUnitID, TextureData {
        .bindID = bindID, .texUnitID = texUnitID,
        .type = GL_TEXTURE_CUBE_MAP, .sampler = TextureSampler::cubemap});
    this->map.emplace(name, texUnitID);
    return texUnitID;
}

uint32_t TextureCollection::addCubemap(const std::string& name, uint32_t width, uint32_t height) {
    const uint32_t texUnitID = this->nextTexUnitID++;

    const GLuint bindID = gfx::texture(gfx::TextureConfig{
        .target = GL_TEXTURE_CUBE_MAP,
        .format = GL_RGB,
        .width = width,
        .height = height,
        .wrap = GL_CLAMP_TO_EDGE,
        .mipmap = true,
    });

    this->idMap.emplace(texUnitID, TextureData {
        .bindID = bindID, .texUnitID = texUnitID,
        .type = GL_TEXTURE_CUBE_MAP, .sampler = TextureSampler::cubemap});
    this->map.emplace(name, texUnitID);
    return texUnitID;
}

const TextureData& TextureCollection::get(uint32_t texUnitID) const {
    return this->idMap.at(texUnitID);
}

const TextureData& TextureCollection::get(const std::string& name) const {
    return this->get(this->map.at(name));
}