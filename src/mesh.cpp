#define NOMINMAX
#include <filesystem>
#include <tuple>
#include <unordered_map>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/fmt.h>
#include "mesh.hpp"

std::string MeshCollection::add(const std::string &filename, const std::string &meshName, bool normalize, bool computeNormals) {
    assert(fs::exists(filename));

    const fs::path textureDir = {"resources/textures"};
    const fs::path modelDir = fs::path(filename).parent_path();
    const std::string name = meshName.empty() ? fs::path(filename).stem().string() : meshName;

    if (this->meshDataMap.count(name)) {
        spdlog::warn("MeshCollection: already has model '{}'", name);
        return name;
    }

    cyTriMesh m;
    m.LoadFromFileObj(filename.c_str());
    m.ComputeBoundingBox();
    if (computeNormals) {
        m.ComputeNormals();
    }
    spdlog::debug("loading '{}' vertices:{}, faces:{}, vertex normals:{}, texture verts:{}, materials:{}",
        filename, m.NV(), m.NF(), m.NVN(), m.NVT(), m.NM());

    // Normalize mesh to fit in a unit cube
    AABB bbox(toEigen(m.GetBoundMin()), toEigen(m.GetBoundMax()));
    if (normalize) {
        const Vector3f boundSize = bbox.max - bbox.min;
        const float maxBoundSize = std::max(std::max(boundSize.x(), boundSize.y()), boundSize.z());
        const float scale = 1.0f / maxBoundSize;
        for (size_t i = 0; i < m.NV(); i++) {
            m.V(i) *= scale;
        }
        bbox = bbox * vec3(scale);
    }

    const size_t vertOffset = this->vertexData.size();
    const size_t elemOffset = this->arrElems.size();
    const size_t nElems = m.NF() * 3;

    // Resize vertex data and element arrays to fit at least as many new vertices as in the obj file
    this->vertexData.resize(vertOffset + m.NV());
    this->arrElems.resize(elemOffset + nElems);

    // Parse materials
    std::vector<uint32_t> meshMaterialIDs(m.NM());
    std::vector<uint32_t> matIDs(m.NF(), 0u);
    for (size_t i = 0; i < m.NM(); i++) {
        uMaterial mat {
            .diffuseColor = vec3(m.M(i).Kd),
            .specularColor = vec3(m.M(i).Ks),
            .ambientColor = vec3(m.M(i).Ka),
            .shininess = m.M(i).Ns
        };

        if (m.M(i).map_Kd) {
            mat.diffuseTexID = this->textures.add(textureDir / fs::path(m.M(i).map_Kd.data));
        }
        if (m.M(i).map_Ks) {
            mat.specularTexID = this->textures.add(textureDir / fs::path(m.M(i).map_Ks.data));
        }

        const uint32_t globalMatID = this->materials.size();
        std::string matName;
        if (m.M(i).name) {
            matName = fmt::format("{}.{}", name, m.M(i).name.data);
        } else {
            matName = fmt::format("{}.mat_{}", name, i);
        }

        meshMaterialIDs[i] = globalMatID;
        this->materials.push_back(mat);

        size_t matFaceOffset = m.GetMaterialFirstFace(i);
        size_t matFaceCount = m.GetMaterialFaceCount(i);
        for (size_t j = 0; j < matFaceCount; j++) {
            matIDs[matFaceOffset + j] = globalMatID;
        }

        this->nameMaterialMap.emplace(matName, globalMatID);
        this->materialNameMap.emplace(globalMatID, matName);
    }

    // Mapping from vertex index to (normalIdx, texIdx)
    std::vector<std::tuple<int, int>> indices(m.NV(), {-1, -1});
    // Remapping (normalIdx, texIdx) -> new vertex index
    std::unordered_map<size_t, size_t> vertRemap({});
    // Map mixed indices to flat indices
    for (size_t i = 0; i < m.NF(); i++) {
        const uint32_t* normalIndices = m.FN(i).v;
        const uint32_t* texIndices = m.FT(i).v;
        const uint32_t* vertIndices = m.F(i).v;
        const uint32_t matID = matIDs.at(i);
        for (size_t j = 0; j < 3; j++) {
            uint32_t vertIdx = vertIndices[j];
            const uint32_t inNormalIdx = normalIndices[j];
            const uint32_t inTexIdx = texIndices[j];
            const uint32_t combinedAttrIdx = cantor(inNormalIdx, inTexIdx);
            if (vertRemap.count(combinedAttrIdx)) {
                // Vertex has already been remapped, so use the stored remapped vertex index
                vertIdx = vertRemap.at(combinedAttrIdx);
            } else {
                const int storedNormalIdx = std::get<0>(indices[vertIdx]);
                const int storedTexIdx = std::get<1>(indices[vertIdx]);
                if (storedNormalIdx != (int)inNormalIdx || storedTexIdx != (int)inTexIdx) {
                    // Vertex either hasn't been stored, or hasn't been remapped
                    const VertexData vertData = {
                        .pos    = toEigen(m.V(vertIdx)),
                        .normal = toEigen(m.VN(inNormalIdx)),
                        .uv     = toEigen(m.VT(inTexIdx)),
                        .matID  = matID
                    };
                    if (storedNormalIdx == -1 || storedTexIdx == -1) {
                        // First encounter with this vertex, store it for the first time
                        indices[vertIdx] = {inNormalIdx, inTexIdx};
                        this->vertexData[vertIdx + vertOffset] = vertData;
                    } else {
                        // Stored index set doesn't match, so we dupe and remap
                        vertIdx = vertRemap.emplace(combinedAttrIdx,
                            this->vertexData.size() - vertOffset).first->second;
                        this->vertexData.push_back(vertData);
                    }
                }
            }
            this->arrElems[elemOffset + i*3 + j] = vertIdx + vertOffset;
        }
    }
    const size_t nVerts = this->vertexData.size() - vertOffset;
    spdlog::debug("\tremapped {} vertex indices", vertRemap.size());

    // Add mesh data
    this->meshDataMap.emplace(name, MeshData{
        .ref = MeshRef{
            .elemCount = (int)nElems,
            .elemByteOffset = elemOffset * sizeof(uint32_t)},
        .materials = meshMaterialIDs,
        .center = bbox.center(),
        .bbox = bbox,
        .vertOffset = vertOffset,
        .vertCount = nVerts
    });

    this->dirty = true;

    return name;
}

std::string MeshCollection::clone(const std::string &meshName, const std::string &newName) {
    const MeshData& meshData = this->meshDataMap.at(meshName);
    const size_t dstVertOffset = this->vertexData.size();
    const size_t srcVertOffset = meshData.vertOffset;
    const size_t dstElemOffset = this->arrElems.size();
    const size_t srcElemOffset = meshData.ref.elemByteOffset / sizeof(uint32_t);

    // Copy vertex and element data, adding new offsets
    this->vertexData.resize(dstVertOffset + meshData.vertCount);
    for (size_t i = 0; i < meshData.vertCount; i++) {
        this->vertexData[i + dstVertOffset] = this->vertexData[i + srcVertOffset];
    }
    this->arrElems.resize(dstElemOffset + meshData.ref.elemCount);
    for (size_t i = 0; i < meshData.ref.elemCount; i++) {
        this->arrElems[i + dstElemOffset] = this->arrElems[i + srcElemOffset] - srcVertOffset + dstVertOffset;
    }

    std::string name = newName;
    if (newName.empty()) {
        name = fmt::format("{}_{}", meshName, this->cloneCount++);
    }

    this->meshDataMap.emplace(name, MeshData{
        .ref = MeshRef{
            .elemCount = meshData.ref.elemCount,
            .elemByteOffset = dstElemOffset * sizeof(uint32_t)},
        .materials = meshData.materials,
        .center = meshData.center,
        .bbox = meshData.bbox,
        .vertOffset = dstVertOffset,
        .vertCount = meshData.vertCount
    });

        this->dirty = true;

    return name;
}

std::string MeshCollection::clone(const std::string &meshName, const uMaterial &mat, const std::string &newName) {
    std::string name = this->clone(meshName, newName);
    size_t newMatID = this->createMaterial(fmt::format("{}.mat_{}", name, this->cloneCount++), mat);
    this->setMaterial(name, newMatID);
    this->dirty = true;
    return name;
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
    GLuint attrib_vPos    = 0u;
    GLuint attrib_vNormal = 1u;
    GLuint attrib_vUV     = 2u;
    GLuint attrib_vMatID  = 3u;

    size_t offset = 0u;

    glEnableVertexAttribArray(attrib_vPos); $gl_err();
    glVertexAttribPointer(attrib_vPos, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData), (void*)(offset)); $gl_err();
    offset += sizeof(VertexData::pos);

    glEnableVertexAttribArray(attrib_vNormal); $gl_err();
    glVertexAttribPointer(attrib_vNormal, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData), (void*)(offset)); $gl_err();
    offset += sizeof(VertexData::normal);

    glEnableVertexAttribArray(attrib_vUV); $gl_err();
    glVertexAttribPointer(attrib_vUV, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData), (void*)(offset)); $gl_err();
    offset += sizeof(VertexData::uv);

    glEnableVertexAttribArray(attrib_vMatID); $gl_err();
    glVertexAttribIPointer(attrib_vMatID, 1, GL_UNSIGNED_INT, sizeof(VertexData), (void*)(offset)); $gl_err();
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

void MeshCollection::bind(cyGLSLProgram& prog, bool includeMaterials) const {
    if (this->dirty) {
        spdlog::warn("MeshCollection::bind() called on dirty MeshCollection");
    }
    // Bind mesh vertex element data buffers
    glBindBuffer(GL_ARRAY_BUFFER, this->vboVerts); $gl_err();
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->eboElems); $gl_err();

    if (includeMaterials) {
        // Bind materials SSBO
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->ssboMaterials); $gl_err();
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, this->ssboMaterials); $gl_err();
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); $gl_err();

        // Bind textures
        this->textures.bind(prog);
    }
}

uMaterial& MeshCollection::setMaterial(const std::string& meshName, size_t matID) {
    MeshData& mesh = this->meshDataMap.at(meshName);

    for (size_t i = mesh.vertOffset; i < mesh.vertOffset + mesh.vertCount; i++) {
        this->vertexData[i].matID = matID;
    }
    mesh.materials = {(uint32_t)matID};

    this->dirty = true;
    return this->materials.at(matID);
}

uMaterial& MeshCollection::setMaterial(const std::string& meshName, const std::string& matName) {
    MeshData& mesh = this->meshDataMap.at(meshName);
    const uint32_t matID = this->nameMaterialMap.at(matName);

    for (size_t i = mesh.vertOffset; i < mesh.vertOffset + mesh.vertCount; i++) {
        this->vertexData[i].matID = matID;
    }
    mesh.materials = {matID};

    this->dirty = true;
    return this->materials.at(matID);
}

uMaterial& MeshCollection::getMaterial(const std::string& meshName) {
    return this->materials.at(this->nameMaterialMap.at(meshName));
}

uMaterial& MeshCollection::getMaterial(size_t matID) {
    return this->materials.at(matID);
}

size_t MeshCollection::createSkyMaterial(const std::string& dirName) {
    this->dirty = true;

    assert(fs::exists(dirName));

    const std::string name = fmt::format("sky_{}", fs::path(dirName).stem().string());
    const uint32_t matID = this->materials.size();
    uMaterial& mat = this->materials.emplace_back();
    mat.reflectionLayer = this->textures.addCubemap(name, dirName);
    
    this->nameMaterialMap[name] = matID;
    this->materialNameMap[matID] = name;
    return matID;
}

size_t MeshCollection::createMaterial(const std::string& name, uMaterial mat) {
    this->dirty = true;

    const uint32_t matID = this->materials.size();

    if (mat.diffuseTexID > -1) {
        mat.diffuseTexID = this->textures.add(name + "_diffuse", mat.diffuseTexID);
    }
    if (mat.specularTexID > -1) {
        mat.specularTexID = this->textures.add(name + "_specular", mat.specularTexID);
    }
    if (mat.flatReflectionTexID > -1) {
        mat.flatReflectionTexID = this->textures.add(name + "_refl", mat.flatReflectionTexID);
    }
    if (mat.normalTexID > -1) {
        mat.normalTexID = this->textures.add(name + "_normal", mat.normalTexID);
    }
    if (mat.displacementTexID > -1) {
        mat.displacementTexID = this->textures.add(name + "_disp", mat.displacementTexID);
    }

    this->materials.push_back(mat);

    this->nameMaterialMap[name] = matID;
    this->materialNameMap[matID] = name;
    return matID;
}

size_t MeshCollection::createMaterial(const std::string &name, size_t matID) {
    return this->createMaterial(name, this->materials.at(matID));
}

size_t MeshCollection::createMaterial(
    const std::string& name,
    const std::string& diffuseTex,
    const std::string& specularTex,
    const std::string& normalTex,
    const std::string& displacementTex)
{
    const uint32_t matID = this->materials.size();
    uMaterial mat;
    if (!diffuseTex.empty()) {
        mat.diffuseTexID = this->textures.add(diffuseTex);
    }
    if (!specularTex.empty()) {
        mat.specularTexID = this->textures.add(specularTex);
    }
    if (!normalTex.empty()) {
        mat.normalTexID = this->textures.add(normalTex);
    }
    if (!displacementTex.empty()) {
        mat.displacementTexID = this->textures.add(displacementTex);
    }
    this->materials.push_back(mat);
    this->nameMaterialMap[name] = matID;
    this->materialNameMap[matID] = name;
    return matID;
}

const TextureData& MeshCollection::getTextureData(int texID) {
    assert(texID >= 0);

    return this->textures.get(texID);
}