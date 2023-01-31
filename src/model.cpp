#include <spdlog/spdlog.h>
#include <model.hpp>

Model::Model(const char* filename, const cyGLSLProgram& prog) {
    mesh.LoadFromFileObj(filename);
    mesh.ComputeBoundingBox();
    this->pivot = toEigen(mesh.GetBoundMax() + mesh.GetBoundMin()) / 2.0f;

    const uint vertDataLen = mesh.NV() * sizeof(cy::Vec3f);

    std::vector<Vector3f> colors(mesh.NV());
    for (uint i = 0; i < mesh.NV(); i++) {
        const Vector3f& vert = toEigen(mesh.V(i));
        colors[i] = hsvToRgb({degrees(angle2D({vert.x(), vert.y()})), 1.0f, 1.0f});
    }
    
    // Create VBO for vertices
    glGenBuffers(1, &this->vertVBO);
    glBindBuffer(GL_ARRAY_BUFFER, this->vertVBO);
    glBufferData(GL_ARRAY_BUFFER, vertDataLen * 2, NULL, GL_STATIC_DRAW);  
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertDataLen, &mesh.V(0));
    glBufferSubData(GL_ARRAY_BUFFER, vertDataLen, vertDataLen, colors.data());

    // Specify vertex attributes
    GLuint attrib_vPos = prog.AttribLocation("vPos");
    glEnableVertexAttribArray(attrib_vPos);
    glVertexAttribPointer(attrib_vPos, 3, GL_FLOAT, GL_FALSE, 0, 0);
    GLuint attrib_vColor = prog.AttribLocation("vColor");
    glEnableVertexAttribArray(attrib_vColor);
    glVertexAttribPointer(attrib_vColor, 3, GL_FLOAT, GL_FALSE, 0, (void*)vertDataLen);
}

const Matrix4f Model::transform() const {
    return identityTransform()
        .translate(this->pos)
        .rotate(euler(this->rot))
        .scale(this->scale)
        .translate(-this->pivot)
        .matrix();
}

void Model::draw(cyGLSLProgram& prog) const {
    glBindBuffer(GL_ARRAY_BUFFER, this->vertVBO);
    prog.SetUniformMatrix4("uTModel", this->transform().data());
    glDrawArrays(GL_POINTS, 0, mesh.NV());
}
