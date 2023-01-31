#version 460

layout(location = 0) in vec4 color;

out vec4 fColor;

float edgeRamp(float x, float f) {
    return clamp(-f * (x - 1.0 + 1.0 / f), 0.0, 1.0);
}

void main() {
    float v = length(gl_PointCoord - 0.5);
    if (v < 0.5) {
        fColor = vec4(color.rgb * edgeRamp(v * 2.0, 10.0), 1.0);
    } else {
        discard;
    }
}
