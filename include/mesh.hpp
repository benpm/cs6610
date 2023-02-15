#pragma once

#include <tuple>
#include <glad/glad.h>
#undef near
#undef far
#include <extmath.hpp>
#include <cyTriMesh.h>
#include <cyGL.h>

// References data inside  mesh collection
struct MeshData {
    GLsizei elemCount; // Number of elements
    size_t elemOffset; // Offset into arrElems
    Vector3f center;   // Center of mesh
};

// Collection of mesh data
class MeshCollection
{
private:
    // Flat, strided vertex array data (pos, color, normal, uv)
    std::vector<Vector3f> arrVerts;
    // Flat element array data (vertex indices for triangles: v0,v1,v2)
    std::vector<uint32_t> arrElems;
    // Map from mesh name to its data offsets
    std::unordered_map<std::string, MeshData> meshDataMap;
    // Indicates if buffers have been built
    mutable bool buffersBuilt = false;
    // Indicates if there are unbuilt changes to the buffers
    mutable bool dirty = false;
    // Vertex data VBO handle
    mutable GLuint vboVerts;
    // Vertex indices EBO handle
    mutable GLuint eboElems;
public:

    // Load and add mesh to the collection. 
    void add(const std::string& filename, const std::string& meshName="", bool normalize=true);
    // Returns the meshdata corresponding to given meshname
    const MeshData& get(const std::string& meshName) const;
    // Builds a VBO and EBO from the data in this collection, or rebuilds if already built
    void build(const cyGLSLProgram& prog) const;
    // Binds the VBO and EBO
    void bind() const;
};
