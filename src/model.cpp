#include <spdlog/spdlog.h>
#include <model.hpp>
#include <camera.hpp>

Model::Model(const char* filename, const cyGLSLProgram& prog) {
    mesh.LoadFromFileObj(filename);
    mesh.ComputeBoundingBox();
    mesh.ComputeNormals();
    this->pivot = toEigen(mesh.GetBoundMax() + mesh.GetBoundMin()) / 2.0f;

    const size_t vertDataLen = mesh.NV() * sizeof(cy::Vec3f);

    std::vector<Vector3f> colors(mesh.NV());
    for (size_t i = 0; i < mesh.NV(); i++) {
        const Vector3f& vert = toEigen(mesh.V(i));
        colors[i] = hsvToRgb({degrees(angle2D({vert.x(), vert.y()})), 1.0f, 1.0f});
    }
    
    // Create VBO for vertices
    glGenBuffers(1, &this->vertVBO);
    glBindBuffer(GL_ARRAY_BUFFER, this->vertVBO);
    glBufferData(GL_ARRAY_BUFFER, vertDataLen * 3, NULL, GL_STATIC_DRAW);  
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertDataLen, &mesh.V(0));
    glBufferSubData(GL_ARRAY_BUFFER, vertDataLen, vertDataLen, colors.data());
    glBufferSubData(GL_ARRAY_BUFFER, vertDataLen * 2, vertDataLen, &mesh.VN(0));

    // Specify vertex attributes

    GLuint attrib_vPos = prog.AttribLocation("vPos");
    glEnableVertexAttribArray(attrib_vPos);
    glVertexAttribPointer(attrib_vPos, 3, GL_FLOAT, GL_FALSE, 0, 0);
    GLuint attrib_vColor = prog.AttribLocation("vColor");
    glEnableVertexAttribArray(attrib_vColor);
    glVertexAttribPointer(attrib_vColor, 3, GL_FLOAT, GL_FALSE, 0, (void*)vertDataLen);
    GLuint attrib_vNormal = prog.AttribLocation("vNormal");
    glEnableVertexAttribArray(attrib_vNormal);
    glVertexAttribPointer(attrib_vNormal, 3, GL_FLOAT, GL_FALSE, 0, (void*)(vertDataLen * 2));

    // Create EBO for triangles
    glGenBuffers(1, &this->triEBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->triEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.NF() * sizeof(cy::TriMesh::TriFace) * 3, &mesh.F(0), GL_STATIC_DRAW);
}

const Matrix4f Model::transform() const {
    return identityTransform()
        .translate(this->pos)
        .rotate(euler(this->rot))
        .scale(this->scale)
        .translate(-this->pivot)
        .matrix();
}

void Model::draw(cyGLSLProgram& prog, const Camera& camera) const {
    glBindBuffer(GL_ARRAY_BUFFER, this->vertVBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->triEBO);
    const Matrix4f tModel = this->transform();
    prog.SetUniformMatrix4("uTModel", tModel.data());
    const Vector3f lightDir =
        (camera.getView() * Vector4f(0.0f, 100.0f, -150.0f, 1.0f)).head<3>().normalized();
    prog.SetUniform3("uLightDir", lightDir.data());
    glDrawElements(GL_TRIANGLES, mesh.NF() * 3, GL_UNSIGNED_INT, 0);
}
