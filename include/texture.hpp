#pragma once

#include <unordered_map>
#include <extmath.hpp>
#include <spng.h>
#include <glad/glad.h>

struct TextureData
{
    GLuint id;
};

class TextureCollection
{
private:
    std::unordered_map<std::string, TextureData> map;
public:
    void add(const std::string& path);
    TextureData get(const std::string& path) const;
};
