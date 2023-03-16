#pragma once

#include <unordered_map>
#include <extmath.hpp>
#include <spng.h>
#include <glad/glad.h>
#include <cyGL.h>

enum class TextureSampler {
    normal,
    shadow
};

struct TextureData
{
    // OpenGL texture name
    GLuint bindID;
    // Index in texture collection (will be split in the future)
    uint32_t texUnitID;
    // Type of texture
    GLenum type;
    // Type of sampler
    TextureSampler sampler = TextureSampler::normal;
};

class TextureCollection
{
private:
    // Next available texture unit ID
    uint32_t nextTexUnitID = 0u;
    std::unordered_map<std::string, uint32_t> map;
    std::unordered_map<uint32_t, TextureData> idMap;
public:
    // Add texture to collection and return its ID (same as texture unit for now)
    uint32_t add(const std::string& path);
    // Adds an already created texture to this collection
    uint32_t add(const std::string& name, GLuint bindID, GLenum type = GL_TEXTURE_2D, TextureSampler sampler = TextureSampler::normal);
    // Adds a cubemap texture, loading texture images from the given directory
    uint32_t addCubemap(const std::string& name, const std::string& dirName);
    // Adds an empty cubemap texture
    uint32_t addCubemap(const std::string& name, uint32_t width, uint32_t height);
    // Bind current texture set to texture units
    void bind(cyGLSLProgram& prog) const;
    // Bind a single texture to a texture unit and a uniform
    void bind(cyGLSLProgram& prog, const std::string& name, const std::string& uniform) const;
    // Gets the texture data associated with the given unit ID
    const TextureData& get(uint32_t texUnitID) const;
    // Gets the texture data associated with the given texture name
    const TextureData& get(const std::string& name) const;
};
