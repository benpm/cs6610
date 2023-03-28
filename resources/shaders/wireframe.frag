#version 460

out vec4 fColor;

in VertexData {
    vec2 uv;
    flat uint matID;
} data_in;

void main() {
    fColor = vec4(0, 1, 0, 1);
}