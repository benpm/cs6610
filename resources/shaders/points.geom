#version 460

layout(triangles) in;
layout(points, max_vertices = 3) out;

in VertexData {
    vec2 uv;
    vec3 normal;
    vec3 position;
    vec3 wnormal;
    vec3 wposition;
    flat uint matID;
    flat uint drawID;
} v_in[];

uniform mat4 uTView;

layout(std430, binding = 0) buffer ModelTransforms {
    mat4 tModel[];
};

void main() {
    for (int i = 0; i < 3; ++i) {
        gl_Position = gl_in[i].gl_Position;
        EmitVertex();
    }
    EndPrimitive();
}