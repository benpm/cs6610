#version 460

in vec3 vPos;
in vec3 vNormal;
in vec3 vUV;
in uint vMatID;

// View space normal
layout(location = 0) out vec3 normal;
// View space position
layout(location = 1) out vec3 position;
// Draw ID
layout(location = 2) flat out uint drawID;
// UV coordinates
layout(location = 3) out vec2 uv;
// Material ID
layout(location = 4) flat out uint matID;
// World space normal
layout(location = 5) out vec3 wnormal;

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
    gl_Position = uTProj * tViewModel * vec4(vPos, 1.0);
    normal = (tViewModel * vec4(vNormal, 0.0)).xyz;
    wnormal = (tModel[gl_DrawID] * vec4(vNormal, 0.0)).xyz;
    position = (tViewModel * vec4(vPos, 1.0)).xyz;
    uv = vUV.xy;
    drawID = gl_DrawID;
    matID = vMatID;
}