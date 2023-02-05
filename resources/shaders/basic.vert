#version 460

layout(location = 0) in vec3 vPos;
layout(location = 1) in vec3 vColor;
layout(location = 2) in vec3 vNormal;

// Vertex color
layout(location = 0) out vec4 color;
// View space normal
layout(location = 1) out vec3 normal;
// View space position
layout(location = 2) out vec3 position;

uniform mat4 uTProj;
uniform mat4 uTView;
uniform mat4 uTModel;

void main()
{
    gl_Position = uTProj * uTView * uTModel * vec4(vPos, 1.0);
    color = vec4(vColor, 1.0);
    normal = (uTView * uTModel * vec4(vNormal, 0.0)).xyz;
    position = (uTView * uTModel * vec4(vPos, 1.0)).xyz;
}