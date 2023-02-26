#version 460

// Vertex position in clip space
in vec3 vPos;

// Vertex position in world space
layout (location = 0) out vec3 position;

// Inverse view-projection matrix
uniform mat4 uTInvViewProj;

void main() {
    position = (uTInvViewProj * vec4(vPos, 0.0)).xyz;
    gl_Position = vec4(vPos, 1.0);
}