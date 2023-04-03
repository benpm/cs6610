#pragma once

#include <bitset>
#include <extmath.hpp>

constexpr size_t chunkSize = 16u;
constexpr size_t chunkCells = chunkSize * chunkSize * chunkSize;

// Adapted from https://0fps.net/2012/07/12/smooth-voxel-terrain-part-2/

class SurfaceNet
{
public:
    // Indices of a cube's vertices
    static const Vector<uint16_t, 24u> cubeVertexIDs;
    // Mapping from cube vertex configuration to edge intersection
    static const std::bitset<256> intersectTable;


};