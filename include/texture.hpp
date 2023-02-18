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
    uint32_t colID;
};

class TextureCollection
{
private:
    uint32_t nextID = 0u;
    std::unordered_map<std::string, TextureData> map;
public:
    // Add texture to collection and return its ID (same as texture unit for now)
    uint32_t add(const std::string& path);
    // Bind current texture set to texture units
    void bind(cyGLSLProgram& prog) const;
};
