#define NOMINMAX
#include <filesystem>
#include <spdlog/spdlog.h>
#include "mesh.hpp"

void MeshCollection::add(const std::string &filename, const std::string &meshName, bool normalize) {
    cyTriMesh m;
    m.LoadFromFileObj(filename.c_str());
    m.ComputeBoundingBox();
    m.ComputeNormals();
    spdlog::debug("loading '{}' vertices:{}, faces:{}, vertex normals:{}, texture verts:{}, materials:{}",
        filename, m.NV(), m.NF(), m.NVN(), m.NVT(), m.NM());

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

    const std::string name = meshName.empty() ? std::filesystem::path(filename).stem().string() : meshName;

    const size_t vertOffset = this->arrVerts.size();
    const size_t triOffset = this->arrElems.size();
    const int nElems = m.NF() * 3;

    // Copy texture coordinates so that they have the same indices as vertices
    std::vector<Vector3f> texCoords(m.NV());
    std::vector<Vector3f> normals(m.NV());
    for (size_t i = 0; i < m.NF(); i++) {
        uint32_t normalIndices[3] = {*m.FN(i).v};
        uint32_t texIndices[3] = {*m.FT(i).v};
        uint32_t vertIndices[3] = {*m.F(i).v};
        for (size_t j = 0; j < 3; j++) {
            assert(vertIndices[j] == normalIndices[j]);
            texCoords.at(vertIndices[j]) = toEigen(m.VT(texIndices[j]));
            normals.at(vertIndices[j]) = toEigen(m.VN(vertIndices[j]));
        }
    }

    // Add vertex data
    this->arrVerts.resize(vertOffset + m.NV() * nVertAttribs);
    for (size_t i = 0; i < m.NV(); i++) {
        // Vertex position
        this->arrVerts[vertOffset + i*nVertAttribs + 0] = toEigen(m.V(i));
        // Vertex color
        this->arrVerts[vertOffset + i*nVertAttribs + 1] = normals[i];
        // Vertex normal
        this->arrVerts[vertOffset + i*nVertAttribs + 2] = normals[i];
        // Vertex UV
        this->arrVerts[vertOffset + i*nVertAttribs + 3] = texCoords[i];
    }

    // Add triangles
    this->arrElems.resize(triOffset + nElems);
    for (size_t i = 0; i < m.NF(); i++) {
        this->arrElems[triOffset + i*3 + 0] = m.F(i).v[0] + (vertOffset / nVertAttribs);
        this->arrElems[triOffset + i*3 + 1] = m.F(i).v[1] + (vertOffset / nVertAttribs);
        this->arrElems[triOffset + i*3 + 2] = m.F(i).v[2] + (vertOffset / nVertAttribs);
    }

    this->meshDataMap.emplace(name, MeshData{
        .elemCount = nElems,
        .elemOffset = triOffset * sizeof(uint32_t),
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
        this->buffersBuilt = true;
    }

    // Populate vertex data VBO
    glBindBuffer(GL_ARRAY_BUFFER, this->vboVerts); $gl_err();
    glBufferData(GL_ARRAY_BUFFER, this->arrVerts.size() * sizeof(Vector3f), this->arrVerts.data(), GL_STATIC_DRAW); $gl_err();

    std::array attribNames = std::to_array({"vPos", "vColor", "vNormal", "vUV"});
    for (size_t i = 0; i < attribNames.size(); i++) {
        GLuint attrib_vPos = prog.AttribLocation(attribNames[i]); $gl_err();
        glEnableVertexAttribArray(attrib_vPos); $gl_err();
        glVertexAttribPointer(attrib_vPos, 3, GL_FLOAT, GL_FALSE,
            sizeof(float) * nVertAttribs * 3u,
            (void*)(sizeof(float) * 3u * i)); $gl_err();
    }

    // Populate triangles elements data EBO
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->eboElems); $gl_err();
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, this->arrElems.size() * sizeof(uint32_t), this->arrElems.data(), GL_STATIC_DRAW); $gl_err();

    this->dirty = false;
}

void MeshCollection::bind() const {
    if (this->dirty) {
        spdlog::warn("MeshCollection::bind() called on dirty MeshCollection");
    }
    glBindBuffer(GL_ARRAY_BUFFER, this->vboVerts); $gl_err();
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->eboElems); $gl_err();
}
