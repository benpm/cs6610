#include <spdlog/spdlog.h>
#include <model.hpp>

Model::Model(const char* filename, const cyGLSLProgram& prog) {
    mesh.LoadFromFileObj(filename);

    const uint n = mesh.NV() * sizeof(cy::Vec3f);

    for (uint i = 0; i < mesh.NV(); i++) {
        mesh.V(i) = mesh.V(i) * 0.1f;
    }
    
    // Create VBO for vertices
    glGenBuffers(1, &this->vbo);
    glBindBuffer(GL_ARRAY_BUFFER, this->vbo);
    glBufferData(GL_ARRAY_BUFFER, n * 2, NULL, GL_STATIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, n, &mesh.V(0));
    glBufferSubData(GL_ARRAY_BUFFER, n, n, &mesh.V(0));

    // Attach vertex attributes
    GLuint attrib_vPos = prog.AttribLocation("vPos");
    glEnableVertexAttribArray(attrib_vPos);
    glVertexAttribPointer(attrib_vPos, 3, GL_FLOAT, GL_FALSE, 0, 0);
    GLuint attrib_vColor = prog.AttribLocation("vColor");
    glEnableVertexAttribArray(attrib_vColor);
    glVertexAttribPointer(attrib_vColor, 3, GL_FLOAT, GL_FALSE, 0, 0);
}

Matrix4f Model::transform() const {
    return Transform(Affine3f(Translation3f(this->pos))).matrix();
}

void Model::draw() const {
    glBindBuffer(GL_ARRAY_BUFFER, this->vbo);
    glDrawArrays(GL_POINTS, 0, mesh.NV());
}
