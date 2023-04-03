#pragma once

#include <bitset>
#include <extmath.hpp>

constexpr size_t chunkSize = 32u;
constexpr size_t chunkCells = chunkSize * chunkSize * chunkSize;

// Adapted from https://0fps.net/2012/07/12/smooth-voxel-terrain-part-2/

class SurfaceNet
{
public:
    std::vector<Vector3f> vertices;
    std::vector<uint32_t> tris;

    SurfaceNet() = default;

    void build(const Vector<float, chunkCells>& data);
};

inline size_t flatIdx(uint16_t x, uint16_t y) {
    return y * chunkSize + x;
}

inline size_t flatIdx(uint16_t x, uint16_t y, uint16_t z) {
    return z * (chunkSize * chunkSize) + flatIdx(x, y);
}
