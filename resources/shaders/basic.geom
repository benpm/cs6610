#version 460

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

in VertexData {
    vec2 uv;
    vec3 normal;
    vec3 position;
    vec3 wnormal;
    vec3 wposition;
    flat uint matID;
    flat uint drawID;
} v_in[];

out VertexData {
    vec2 uv;
    vec3 normal;
    vec3 position;
    vec3 wnormal;
    vec3 wposition;
    flat uint matID;
    flat uint drawID;
} v_out;

uniform mat4 uTView;

layout(std430, binding = 0) buffer ModelTransforms {
    mat4 tModel[];
};

void main() {
    const mat3 m = mat3(tModel[v_in[0].drawID]);
    const mat3 v = mat3(uTView);
    const vec3 wnormal = m * normalize(cross(
        v_in[1].position - v_in[0].position,
        v_in[2].position - v_in[0].position));
    const vec3 normal = normalize(v * wnormal);

    for (int i = 0; i < 3; ++i) {
        v_out.normal = normal;
        v_out.position = v_in[i].position;
        v_out.wnormal = wnormal;
        v_out.wposition = v_in[i].wposition;
        v_out.uv = v_in[i].uv;
        v_out.matID = v_in[i].matID;
        gl_Position = gl_in[i].gl_Position;
        EmitVertex();
    }
    EndPrimitive();
}