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
    const vec4 wnormal = vec4(normalize(cross(
        v_in[1].wposition - v_in[0].wposition,
        v_in[2].wposition - v_in[0].wposition)), 0.0);
    const vec3 normal = normalize((uTView * wnormal).xyz);

    for (int i = 0; i < 3; ++i) {
        v_out.normal = normal;
        v_out.position = v_in[i].position;
        v_out.wnormal = wnormal.xyz;
        v_out.wposition = v_in[i].wposition;
        v_out.uv = v_in[i].uv;
        v_out.matID = v_in[i].matID;
        gl_Position = gl_in[i].gl_Position;
        EmitVertex();
    }
    EndPrimitive();
}