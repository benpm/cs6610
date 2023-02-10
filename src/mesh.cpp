#include <filesystem>
#include <spdlog/spdlog.h>
#include "mesh.hpp"

void MeshCollection::add(const std::string &filename, const std::string &meshName, bool normalize) {
    cyTriMesh m;
    m.LoadFromFileObj(filename.c_str());
    m.ComputeBoundingBox();
    m.ComputeNormals();

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

    // Add vertex data
    this->arrVerts.resize(vertOffset + m.NV() * 3);
    for (size_t i = 0; i < m.NV(); i++) {
        // Vertex position
        this->arrVerts[vertOffset + i*3 + 0] = toEigen(m.V(i));
        // Vertex color
        this->arrVerts[vertOffset + i*3 + 1] = toEigen(m.VN(i));
        // Vertex normal
        this->arrVerts[vertOffset + i*3 + 2] = toEigen(m.VN(i));
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
        .elemOffset = triOffset * sizeof(uint32_t)
    });

    this->dirty = true;
}

const MeshData& MeshCollection::get(const std::string &meshName) const {
    return this->meshDataMap.at(meshName);
}

void MeshCollection::build(const cyGLSLProgram& prog) const {
    if (!this->buffersBuilt) {
        glGenBuffers(1, &this->vboVerts);
        glGenBuffers(1, &this->eboElems);
        this->buffersBuilt = true;
    }

    // Populate vertex data VBO
    glBindBuffer(GL_ARRAY_BUFFER, this->vboVerts);
    glBufferData(GL_ARRAY_BUFFER, this->arrVerts.size() * sizeof(Vector3f), this->arrVerts.data(), GL_STATIC_DRAW);

    GLuint attrib_vPos = prog.AttribLocation("vPos");
    glEnableVertexAttribArray(attrib_vPos);
    glVertexAttribPointer(attrib_vPos, 3, GL_FLOAT, GL_FALSE,
        sizeof(float) * nVertAttribs * 3u, (void*)(sizeof(float) * nVertAttribs * 0u));
    GLuint attrib_vColor = prog.AttribLocation("vColor");
    glEnableVertexAttribArray(attrib_vColor);
    glVertexAttribPointer(attrib_vColor, 3, GL_FLOAT, GL_FALSE,
        sizeof(float) * nVertAttribs * 3u, (void*)(sizeof(float) * nVertAttribs * 1u));
    GLuint attrib_vNormal = prog.AttribLocation("vNormal");
    glEnableVertexAttribArray(attrib_vNormal);
    glVertexAttribPointer(attrib_vNormal, 3, GL_FLOAT, GL_FALSE,
        sizeof(float) * nVertAttribs * 3u, (void*)(sizeof(float) * nVertAttribs * 2u));

    // Populate triangles elements data EBO
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->eboElems);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, this->arrElems.size() * sizeof(uint32_t), this->arrElems.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    this->dirty = false;
}

void MeshCollection::bind() const {
    if (this->dirty) {
        spdlog::warn("MeshCollection::bind() called on dirty MeshCollection");
    }
    glBindBuffer(GL_ARRAY_BUFFER, this->vboVerts);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->eboElems);
}
