#pragma once

#include <tuple>
#include <ostream>
#include <spdlog/spdlog.h>
#include <spdlog/formatter.h>
#include <texture.hpp>
#include <glad/glad.h>
#undef near
#undef far
#include <extmath.hpp>
#include <cyTriMesh.h>
#include <cyGL.h>

struct VertexData {
    // Number of vertex attributes
    static constexpr uint32_t attributes = 4u;
    // Names of vertex attributes
    static constexpr std::array<const char*, attributes> attributeNames {
        "vPos",
        "vNormal",
        "vUV",
        "vMatID"
    };
    
    Vector3f pos;
    Vector3f normal;
    Vector3f uv;
    uint32_t matID;
};

struct uMaterial {
    static constexpr std::size_t page_size = 65536u;
    alignas(16) Vector3f diffuseColor = {1.0f, 1.0f, 1.0f};
    alignas(16) Vector3f specularColor = {1.0f, 1.0f, 1.0f};
    alignas(16) Vector3f ambientColor = {1.0f, 1.0f, 1.0f};
    alignas(16) Vector3f emissionColor = {0.0f, 0.0f, 0.0f};
    float shininess = 35.0f;
    float specularFactor = 1.0f;
    float ambientFactor = 0.05f;
    float emissionFactor = 0.0f;
    int diffuseTexID = -1;
    int specularTexID = -1;
    int reflectionTexID = -1;
    int flatReflectionTexID = -1;
};

template<> struct fmt::formatter<uMaterial> {
    constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin()) {
        return ctx.end();
    }

    template <typename FormatContext>
    auto format(const uMaterial& o, FormatContext& ctx) -> decltype(ctx.out()) {
        return format_to(ctx.out(),
            "material(diffuseColor={}, specularColor={}, ambientColor={}, shininess={}, specularFactor={}, ambientFactor={}, diffuseTexID={}, specularTexID={})",
            o.diffuseColor, o.specularColor, o.ambientColor, o.shininess, o.specularFactor, o.ambientFactor, o.diffuseTexID, o.specularTexID
        );
    }
};

// References data inside mesh collection
struct MeshRef {
    GLsizei elemCount; // Number of elements
    size_t elemByteOffset; // Byte offset into arrElems
};

// Mesh data
struct MeshData {
    // Reference information: element counts and offsets
    MeshRef ref;
    // List of material indices, references materials in MeshCollection::materials
    std::vector<uint32_t> materials;
    // Center of mesh
    Vector3f center;
    // Bounding box of mesh
    AABB bbox;
    // Offset into vertex data
    size_t vertOffset;
    // Number of vertices
    size_t vertCount;
};

// Collection of mesh and material data
class MeshCollection
{
private:
    // Counter for naming clones
    uint32_t cloneCount = 0u;
    // Flat, strided vertex array data (pos, color, normal, uv)
    std::vector<VertexData> vertexData;
    // Flat element array data (vertex indices for triangles: v0,v1,v2)
    std::vector<uint32_t> arrElems;
    // Map from mesh name to its data offsets
    std::unordered_map<std::string, MeshData> meshDataMap;
    // All materials for all stored meshes, referenced by MeshData::materials
    std::vector<uMaterial> materials = {uMaterial{}};
    // Map material names -> material indices
    std::unordered_map<std::string, size_t> nameMaterialMap = {{"DEFAULT", 0u}};
    // Map material indices -> material names
    std::unordered_map<size_t, std::string> materialNameMap = {{0u, "DEFAULT"}};
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
    // All textures for all stored materials
    TextureCollection textures;
    
    // Load and add mesh to the collection, returns name
    std::string add(const std::string& filename, const std::string& meshName="", bool normalize=true, bool computeNormals=true);
    // Clones mesh with given name, returns new name
    std::string clone(const std::string& meshName, const std::string& newName="");
    // Clones mesh with given name, assigning to a new material, returns new name
    std::string clone(const std::string& meshName, const uMaterial& mat, const std::string& newName="");
    // Returns the meshdata corresponding to given meshname
    const MeshData& get(const std::string& meshName) const;
    // Builds a VBO and EBO from the data in this collection, or rebuilds if already built
    void build(const cyGLSLProgram& prog) const;
    // Binds vertex data, element data, materials, textures
    void bind(cyGLSLProgram& prog, bool includeMaterials=true) const;
    // Assigns a given material ID to all vertices in a mesh, returns material
    uMaterial& setMaterial(const std::string& meshName, size_t matID);
    // Assigns a custom material (a named material) to a all vertices in a mesh
    uMaterial& setMaterial(const std::string& meshName, const std::string& matName);
    // Gets material by name
    uMaterial& getMaterial(const std::string& meshName);
    // Gets material by ID
    uMaterial& getMaterial(size_t matID);
    // Creates a new sky material (will be named "sky_<dirName>")
    size_t createSkyMaterial(const std::string& dirName);
    // Creates a new buffer material (material which is meant to be used as a framebuffer target)
    uMaterial& createBufferMaterial(const std::string& name, uint32_t width, uint32_t height);
    // Creates a new material, inferring from given, returns ID
    size_t createMaterial(const std::string& name, uMaterial mat);
    // Creates a new material as a copy of the given material, returns ID
    size_t createMaterial(const std::string& name, size_t matID);
    // Gets the texturedata with given ID (the IDs from the materials)
    const TextureData& getTextureData(int texID);
};
