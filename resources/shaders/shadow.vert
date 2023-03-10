#version 460

in vec3 vPos;
in vec3 vNormal;
in vec3 vUV;
in uint vMatID;

layout(location = 0) out vec4 fragPos;

uniform mat4 uTProj;
uniform mat4 uTView;

// SSBO for transform matrices
layout(std430, binding = 0) buffer ModelTransforms
{
    mat4 tModel[];
};

void main()
{
    const mat4 tViewModel = uTView * tModel[gl_DrawID];
    fragPos = tViewModel * vec4(vPos, 1.0);
    gl_Position = uTProj * fragPos;
}