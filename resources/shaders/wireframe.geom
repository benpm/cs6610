#version 460

layout(triangles) in;
layout(line_strip, max_vertices = 4) out;

in VertexData {
    vec2 uv;
    vec3 normal;
    vec3 position;
    vec3 wnormal;
    vec3 wposition;
    flat uint matID;
    flat uint drawID;
} data_in[];

out VertexData {
    vec2 uv;
    flat uint matID;
} data_out;

void main() {
    for (int i = 0; i < 4; ++i) {
        int j = i % 3;
        data_out.uv = data_in[j].uv;
        data_out.matID = data_in[j].matID;
        gl_Position = gl_in[j].gl_Position;
        EmitVertex();
    }
    EndPrimitive();
}