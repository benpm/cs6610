#pragma once

#include <tuple>
#include <texture.hpp>
#include <glad/glad.h>
#undef near
#undef far
#include <extmath.hpp>
#include <cyTriMesh.h>
#include <cyGL.h>

struct VertexData {
    // Number of vertex attributes
    static constexpr uint32_t attributes = 5u;
    // Names of vertex attributes
    static constexpr std::array<const char*, attributes> attributeNames {
        "vPos",
        "vColor",
        "vNormal",
        "vUV",
        "vMatID"
    };
    
    Vector3f pos;
    Vector3f color;
    Vector3f normal;
    Vector3f uv;
    alignas(4) uint32_t matID;
};

struct uMaterial {
    static constexpr std::size_t page_size = 65536u;
    alignas(16) Vector3f diffuseColor = {1.0f, 1.0f, 1.0f};
    alignas(16) Vector3f specularColor = {1.0f, 1.0f, 1.0f};
    alignas(16) Vector3f ambientColor = {1.0f, 1.0f, 1.0f};
    float shininess = 35.0f;
    float specularFactor = 2.0f;
    float ambientFactor = 0.05f;
    uint32_t diffuseTexID = 0u;
    uint32_t specularTexID = 0u;
};

// References data inside mesh collection
struct MeshRef {
    GLsizei elemCount; // Number of elements
    size_t elemOffset; // Offset into arrElems
};

// Mesh data
struct MeshData {
    // Reference information: element counts and offsets
    MeshRef ref;
    // List of material indices, references materials in MeshCollection::materials
    std::vector<uint32_t> materials;
    // Center of mesh
    Vector3f center;
};

// Collection of mesh data
class MeshCollection
{
private:
    // Flat, strided vertex array data (pos, color, normal, uv)
    std::vector<VertexData> vertexData;
    // Flat element array data (vertex indices for triangles: v0,v1,v2)
    std::vector<uint32_t> arrElems;
    // All textures for all stored materials
    TextureCollection textures;
    // Map from mesh name to its data offsets
    std::unordered_map<std::string, MeshData> meshDataMap;
    // All materials for all stored meshes, referenced by MeshData::materials
    std::vector<uMaterial> materials;
    // Indicates if buffers have been built
    mutable bool buffersBuilt = false;
    // Indicates if there are unbuilt changes to the buffers
    mutable bool dirty = false;
    // Vertex data VBO handle
    mutable GLuint vboVerts;
    // Vertex indices EBO handle
    mutable GLuint eboElems;
    // Materials SSBO handle
    mutable GLuint ssboMaterials;
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
