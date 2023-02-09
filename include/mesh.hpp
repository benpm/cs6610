#pragma once

#include <glad/glad.h>
#include <extmath.hpp>
#include <cyTriMesh.h>
#include <cyGL.h>

class MeshCollection
{
public:
    // Flat, strided vertex array data (pos, color, normal)
    std::vector<Vector3f> arrVerts;
    // Flat element array data (vertex indices for triangles: v0,v1,v2)
    std::vector<uint32_t> arrElems;
    // Element counts
    std::vector<GLsizei> vCounts;
    // Element offsets
    std::vector<size_t> vOffsets;
    // Map from mesh handle to its offset in the vertex data
    std::unordered_map<size_t, size_t> handleOffsetMap;
    // Map from mesh name to its handle
    std::unordered_map<std::string, size_t> nameHandleMap;

    // Load and add mesh to the collection. 
    void add(const std::string& filename, const std::string& meshName="", bool normalize=true);
    size_t handle(const std::string& meshName);
};