#version 460

// Vertex color
layout(location = 0) in vec4 color;
// View space normal
layout(location = 1) in vec3 normal;
// View space position
layout(location = 2) in vec3 position;

// Fragment color
out vec4 fColor;

// Blinn-phong: light direction in view-space
uniform vec3 uLightDir;

// https://www.desmos.com/calculator/89ahzn84ek
float edgeRamp(float x, float f, float d) {
    return clamp(1.0 - f * (x - 1.0 + d + 1.0 / f), 0.0, 1.0);
}

void main() {
    // fColor = vec4(normalize(normal), 1.0);
    vec3 C = vec3(dot(normalize(normal), uLightDir));
    fColor = vec4(C, 1.0);
}
