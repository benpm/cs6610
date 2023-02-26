#version 460

layout (location = 0) in vec3 position;

out vec4 fColor;

uniform samplerCube uSkyTex;

void main() {
    fColor = texture(uSkyTex, position);
}