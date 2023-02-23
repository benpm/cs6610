#pragma once

#include <unordered_map>
#include <extmath.hpp>
#include <spng.h>
#include <glad/glad.h>
#include <cyGL.h>

struct TextureData
{
    // OpenGL texture name
    GLuint bindID;
    // Index in texture collection (will be split in the future)
    uint32_t texUnitID;
    // Type of texture
    GLenum type;
};

class TextureCollection
{
private:
    // Next available texture unit ID
    uint32_t nextTexUnitID = 0u;
    std::unordered_map<std::string, TextureData> map;
public:
    // Add texture to collection and return its ID (same as texture unit for now)
    uint32_t add(const std::string& path);
    // Adds an already created texture to this collection
    uint32_t add(const std::string& name, GLuint bindID);
    // Adds a cubemap texture
    uint32_t addCubemap(const std::string& name, const std::array<std::string, 6u>& paths);
    // Bind current texture set to texture units
    void bind(cyGLSLProgram& prog) const;
};
