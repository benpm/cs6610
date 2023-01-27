#version 420

layout(location = 0) in vec3 vPos;
layout(location = 1) in vec4 vColor;

layout(location = 0) out vec4 color;

uniform mat4 uMVP;

void main()
{
    gl_Position = uMVP * vec4(vPos, 1.0);
    color = vColor;
}