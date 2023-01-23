#version 420

layout(location = 0) in vec3 vPos;
layout(location = 1) in vec4 vColor;

out vec4 oColor;

uniform mat4 uMVP;

void main()
{
    gl_Position = uMVP * vec4(vPos, 1.0);
    oColor = vColor;
}