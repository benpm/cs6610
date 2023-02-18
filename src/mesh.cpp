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
    const size_t triOffset = this->arrElems.size();
    const int nElems = m.NF() * 3;

    // Store all combinations of tex,normal indices for each vertex index
    std::vector<std::tuple<int, int>> indices(m.NV(), {-1, -1});
    std::unordered_map<std::tuple<uint32_t, uint32_t>, size_t> vertRemap({});
    for (size_t i = 0; i < m.NF(); i++) {
        uint32_t normalIndices[3] = {*m.FN(i).v};
        uint32_t texIndices[3] = {*m.FT(i).v};
        uint32_t vertIndices[3] = {*m.F(i).v};
        for (size_t j = 0; j < 3; j++) {
            const VertexData vertData = {
                .pos = toEigen(m.V(vertIndices[j])),
                .color = {1.0f, 1.0f, 1.0f},
                .normal = toEigen(m.VN(normalIndices[j])),
                .uv = toEigen(m.VT(texIndices[j])),
                .matID = m.GetMaterialIndex(i)
            };
            uint32_t inNormalIdx = normalIndices[j];
            uint32_t inTexIdx = texIndices[j];
            int storedNormalIdx = std::get<0>(indices[vertIndices[j]]);
            int storedTexIdx = std::get<1>(indices[vertIndices[j]]);
            if (storedNormalIdx == -1) {
                std::get<0>(indices[vertIndices[j]]) = inNormalIdx;
                std::get<1>(indices[vertIndices[j]]) = inTexIdx;
            } else {
                indices[vertIndices[j]] = {inNormalIdx, inTexIdx};
                if (storedNormalIdx != (int)inNormalIdx || storedTexIdx != (int)inTexIdx) {
                    std::tuple<uint32_t, uint32_t> k = {inNormalIdx, inTexIdx};
                    if (!vertRemap.count(k)) {
                        // Duplicate vertex
                        vertRemap[k] = this->vertexData.size();
                        this->vertexData.push_back(vertData);
                    }
                } else {
                    this->vertexData[vertIndices[j]] = vertData;
                }
            }
        }
    }

    // Add triangles
    this->arrElems.resize(triOffset + nElems);
    for (size_t i = 0; i < m.NF(); i++) {
        this->arrElems[triOffset + i*3 + 0] = m.F(i).v[0] + vertOffset;
        this->arrElems[triOffset + i*3 + 1] = m.F(i).v[1] + vertOffset;
        this->arrElems[triOffset + i*3 + 2] = m.F(i).v[2] + vertOffset;
    }

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
            .elemCount = nElems,
            .elemOffset = triOffset * sizeof(uint32_t),
            .center = pivot},
        .materials = meshMaterialIDs
    });

    this->dirty = true;
}

const MeshRef& MeshCollection::get(const std::string &meshName) const {
    return this->meshDataMap.at(meshName).ref;
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
}
