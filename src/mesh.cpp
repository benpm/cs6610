#define NOMINMAX
#include <filesystem>
#include <tuple>
#include <unordered_map>
#include <spdlog/spdlog.h>
#include "mesh.hpp"

void MeshCollection::add(const std::string &filename, const std::string &meshName, bool normalize) {
    cyTriMesh m;
    m.LoadFromFileObj(filename.c_str());
    m.ComputeBoundingBox();
    m.ComputeNormals();
    spdlog::debug("loading '{}' vertices:{}, faces:{}, vertex normals:{}, texture verts:{}, materials:{}",
        filename, m.NV(), m.NF(), m.NVN(), m.NVT(), m.NM());

    // Normalize mesh to fit in a unit cube
    Vector3f pivot = toEigen(m.GetBoundMax() + m.GetBoundMin()) / 2.0f;
    if (normalize) {
        const Vector3f boundMin = toEigen(m.GetBoundMin());
        const Vector3f boundMax = toEigen(m.GetBoundMax());
        const Vector3f boundSize = boundMax - boundMin;
        const float maxBoundSize = std::max(std::max(boundSize.x(), boundSize.y()), boundSize.z());
        const float scale = 1.0f / maxBoundSize;
        for (size_t i = 0; i < m.NV(); i++) {
            m.V(i) *= scale;
        }
        pivot *= scale;
    }

    const std::filesystem::path textureDir = {"resources/textures"};
    const std::filesystem::path modelDir = std::filesystem::path(filename).parent_path();
    const std::string name = meshName.empty() ? std::filesystem::path(filename).stem().string() : meshName;

    const size_t vertOffset = this->vertexData.size();
    const size_t elemOffset = this->arrElems.size();
    const size_t nElems = m.NF() * 3;

    // Resize vertex data and element arrays to fit at least as many new vertices as in the obj file
    this->vertexData.resize(vertOffset + m.NV());
    this->arrElems.resize(elemOffset + nElems);

    // Mapping from vertex index to (normalIdx, texIdx)
    std::vector<std::tuple<int, int>> indices(m.NV(), {-1, -1});
    // Map from text,normal indices to vertex index
    std::unordered_map<size_t, size_t> vertRemap({});
    // Map mixed indices to flat indices
    for (size_t i = 0; i < m.NF(); i++) {
        const uint32_t* normalIndices = m.FN(i).v;
        const uint32_t* texIndices = m.FT(i).v;
        const uint32_t* vertIndices = m.F(i).v;
        const int matID = std::max(0, m.GetMaterialIndex(i));
        for (size_t j = 0; j < 3; j++) {
            uint32_t vertIdx = vertIndices[j];
            const uint32_t inNormalIdx = normalIndices[j];
            const uint32_t inTexIdx = texIndices[j];
            if (vertRemap.count(vertIdx)) {
                // Vertex has already been remapped, so use the stored remapped vertex index
                vertIdx = vertRemap.at(vertIdx);
            } else {
                // Vertex either hasn't been stored, or hasn't been remapped
                const int storedNormalIdx = std::get<0>(indices[vertIdx]);
                const int storedTexIdx = std::get<1>(indices[vertIdx]);
                const VertexData vertData = {
                    .pos    = toEigen(m.V(vertIdx)),
                    .color  = {1.0f, 1.0f, 1.0f},
                    .normal = toEigen(m.VN(inNormalIdx)),
                    .uv     = toEigen(m.VT(inTexIdx)),
                    .matID  = (uint32_t)matID
                };
                if (storedNormalIdx == -1 || storedTexIdx == -1) {
                    // First encounter with this vertex
                    indices[vertIdx] = {inNormalIdx, inTexIdx};
                    this->vertexData[vertIdx + vertOffset] = vertData;
                } else {
                    if (storedNormalIdx != (int)inNormalIdx || storedTexIdx != (int)inTexIdx) {
                        // Stored index set doesn't match, so we dupe and remap
                        vertIdx = vertRemap.emplace(vertIdx, this->vertexData.size() - vertOffset).first->second;
                        this->vertexData.push_back(vertData);
                    }
                }
            }
            this->arrElems[elemOffset + i*3 + j] = vertIdx + vertOffset;
        }
    }
    spdlog::debug("\tremapped {} vertex indices", vertRemap.size());

    // Parse materials
    std::vector<uint32_t> meshMaterialIDs(m.NM());
    for (size_t i = 0; i < m.NM(); i++) {
        uMaterial mat {
            .diffuseColor = vec3(m.M(i).Kd),
            .specularColor = vec3(m.M(i).Ks),
            .ambientColor = vec3(m.M(i).Ka),
            .shininess = m.M(i).Ns
        };

        if (m.M(i).map_Kd) {
            mat.diffuseTexID = this->textures.add(textureDir / std::filesystem::path(m.M(i).map_Kd.data));
        }
        if (m.M(i).map_Ks) {
            mat.specularTexID = this->textures.add(textureDir / std::filesystem::path(m.M(i).map_Ks.data));
        }

        meshMaterialIDs[i] = this->materials.size();
        this->materials.push_back(mat);
    }

    // Add mesh data
    this->meshDataMap.emplace(name, MeshData{
        .ref = MeshRef{
            .elemCount = (int)nElems,
            .elemOffset = elemOffset * sizeof(uint32_t)},
        .materials = meshMaterialIDs,
        .center = pivot
    });

    this->dirty = true;
}

const MeshData& MeshCollection::get(const std::string &meshName) const {
    return this->meshDataMap.at(meshName);
}

void MeshCollection::build(const cyGLSLProgram& prog) const {
    if (!this->buffersBuilt) {
        glGenBuffers(1, &this->vboVerts); $gl_err();
        glGenBuffers(1, &this->eboElems); $gl_err();
        glGenBuffers(1, &this->ssboMaterials); $gl_err();
        this->buffersBuilt = true;
    }

    // Populate vertex data VBO
    glBindBuffer(GL_ARRAY_BUFFER, this->vboVerts); $gl_err();
    glBufferData(GL_ARRAY_BUFFER,
        this->vertexData.size() * sizeof(VertexData), this->vertexData.data(), GL_STATIC_DRAW); $gl_err();

    // Specify vertex attributes
    GLuint attrib_vPos    = prog.AttribLocation("vPos"); $gl_err();
    GLuint attrib_vColor  = prog.AttribLocation("vColor"); $gl_err();
    GLuint attrib_vNormal = prog.AttribLocation("vNormal"); $gl_err();
    GLuint attrib_vUV     = prog.AttribLocation("vUV"); $gl_err();
    GLuint attrib_vMatID  = prog.AttribLocation("vMatID"); $gl_err();

    size_t offset = 0u;

    glEnableVertexAttribArray(attrib_vPos); $gl_err();
    glVertexAttribPointer(attrib_vPos, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData), (void*)(offset)); $gl_err();
    offset += sizeof(VertexData::pos);

    glEnableVertexAttribArray(attrib_vColor); $gl_err();
    glVertexAttribPointer(attrib_vColor, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData), (void*)(offset)); $gl_err();
    offset += sizeof(VertexData::color);

    glEnableVertexAttribArray(attrib_vNormal); $gl_err();
    glVertexAttribPointer(attrib_vNormal, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData), (void*)(offset)); $gl_err();
    offset += sizeof(VertexData::normal);

    glEnableVertexAttribArray(attrib_vUV); $gl_err();
    glVertexAttribPointer(attrib_vUV, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData), (void*)(offset)); $gl_err();
    offset += sizeof(VertexData::uv);

    glEnableVertexAttribArray(attrib_vMatID); $gl_err();
    glVertexAttribPointer(attrib_vMatID, 1, GL_UNSIGNED_INT, GL_FALSE, sizeof(VertexData), (void*)(offset)); $gl_err();
    offset += sizeof(VertexData::matID);

    // Populate triangles elements data EBO
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->eboElems); $gl_err();
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
        this->arrElems.size() * sizeof(uint32_t), this->arrElems.data(), GL_STATIC_DRAW); $gl_err();
    
    // Populate materials SSBO
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboMaterials); $gl_err();
    glBufferData(GL_SHADER_STORAGE_BUFFER,
        this->materials.size() * sizeof(uMaterial), this->materials.data(), GL_DYNAMIC_DRAW); $gl_err();
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, this->ssboMaterials); $gl_err();
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); $gl_err();

    this->dirty = false;
    spdlog::debug("built mesh collection with {} vertices and {} elements", this->vertexData.size(), this->arrElems.size());
}

void MeshCollection::bind() const {
    if (this->dirty) {
        spdlog::warn("MeshCollection::bind() called on dirty MeshCollection");
    }
    // Bind mesh vertex element data buffers
    glBindBuffer(GL_ARRAY_BUFFER, this->vboVerts); $gl_err();
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->eboElems); $gl_err();

    // Bind materials SSBO
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboMaterials); $gl_err();
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, this->ssboMaterials); $gl_err();
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); $gl_err();

    // Bind textures
    this->textures.bind();
}
