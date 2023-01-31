#version 460

layout(location = 0) in vec4 color;

out vec4 fColor;

// https://www.desmos.com/calculator/89ahzn84ek
float edgeRamp(float x, float f, float d) {
    return clamp(1.0 - f * (x - 1.0 + d + 1.0 / f), 0.0, 1.0);
}

void main() {
    float v = length(gl_PointCoord - 0.5);
    float a = edgeRamp(v * 2.0, 10.0, 0.05);
    if (a < 1e-9) {
        discard;
    } else {
        fColor = vec4(color.rgb * edgeRamp(v * 2.0, 10.0, 0.2), a);
    }
}
