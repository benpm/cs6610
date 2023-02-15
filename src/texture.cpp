#include <filesystem>
#include <texture.hpp>
#include <glad/glad.h>

void TextureCollection::add(const std::string& path) {
    spng_ctx* ctx = spng_ctx_new(0);
    if (!ctx) {
        throw std::runtime_error("Failed to create libspng context");
    }

    FILE* f = fopen(path.c_str(), "rb");
    spng_set_png_file(ctx, f);
    spng_ihdr header;
    spng_get_ihdr(ctx, &header);

    size_t len;
    spng_decoded_image_size(ctx, SPNG_FMT_RGBA8, &len);
    std::vector<uint8_t> data(len);
    spng_decode_image(ctx, data.data(), data.size(), SPNG_FMT_RGBA8, 0);

    GLuint id;
    glGenTextures(1, &id);
    glBindTexture(GL_TEXTURE_2D, id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, header.width, header.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data.data());
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    const std::string name = std::filesystem::path(path).stem().string();

    this->map.emplace(name, TextureData {.id = id});
    spng_ctx_free(ctx);
}

TextureData TextureCollection::get(const std::string& path) const {
    return this->map.at(path);
}