#version 460

in VertexData {
    vec2 uv;
    vec3 normal;
    vec3 position;
    vec3 wnormal;
    vec3 wposition;
    flat uint matID;
    flat uint drawID;
} data_in;

uniform vec3 uLightPos;
uniform float uFarPlane;
uniform mat4 uTProj;

void main() {
    gl_FragDepth = length((uTProj * vec4(data_in.position, 1.0)).xyz) / uFarPlane;
} 