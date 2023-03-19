#version 460

in vec4 fragPos;

uniform vec3 uLightPos;
uniform float uFarPlane;

void main() {
    gl_FragDepth = fragPos.z / uFarPlane;
} 