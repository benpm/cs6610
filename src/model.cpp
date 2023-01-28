#include <spdlog/spdlog.h>
#include <model.hpp>

Model::Model(const char* filename, const cyGLSLProgram& prog) {
    mesh.LoadFromFileObj(filename);

    const uint vertDataLen = mesh.NV() * sizeof(cy::Vec3f);
    
    // Create VBO for vertices
    glGenBuffers(1, &this->vertVBO);
    glBindBuffer(GL_ARRAY_BUFFER, this->vertVBO);
    glBufferData(GL_ARRAY_BUFFER, vertDataLen * 2, NULL, GL_STATIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertDataLen, &mesh.V(0));
    glBufferSubData(GL_ARRAY_BUFFER, vertDataLen, vertDataLen, &mesh.V(0));   

    // Specify vertex attributes
    GLuint attrib_vPos = prog.AttribLocation("vPos");
    glEnableVertexAttribArray(attrib_vPos);
    glVertexAttribPointer(attrib_vPos, 3, GL_FLOAT, GL_FALSE, 0, 0);
    GLuint attrib_vColor = prog.AttribLocation("vColor");
    glEnableVertexAttribArray(attrib_vColor);
    glVertexAttribPointer(attrib_vColor, 3, GL_FLOAT, GL_FALSE, 0, 0);
}

const Matrix4f Model::transform() const {
    return Transform(Affine3f(
        Translation3f(this->pos) *
        AlignedScaling3f(this->scale)
    )).matrix();
}

void Model::draw(cyGLSLProgram& prog) const {
    glBindBuffer(GL_ARRAY_BUFFER, this->vertVBO);
    prog.SetUniformMatrix4("uTModel", this->transform().data());
    glDrawArrays(GL_POINTS, 0, mesh.NV());
}
