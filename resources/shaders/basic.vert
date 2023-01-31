#version 460

layout(location = 0) in vec3 vPos;
layout(location = 1) in vec3 vColor;

layout(location = 0) out vec4 color;

uniform mat4 uTProjView;
uniform mat4 uTModel;

void main()
{
    vec4 p = uTProjView * uTModel * vec4(vPos, 1.0);
    gl_Position = p;
    float f = clamp((1.0 / (p.z * 0.5f)), 0.0, 1.0);
    gl_PointSize = 15.0 * f;
    color = vec4(mix(vColor, vec3(0.0), (1.0 - f)), 1.0);
}