#version 460

in vec3 vPos;

layout(location = 0) out vec4 color;

uniform mat4 uTProj;
uniform mat4 uTView;

// SSBO for transform matrices
layout(std430, binding = 0) buffer ModelTransforms
{
    mat4 tModel[];
};

// SSBO for colors
layout(std430, binding = 1) buffer ArrowColors
{
    vec4 uColor[];
};

void main()
{
    const mat4 tViewModel = uTView * tModel[gl_InstanceID];
    gl_Position = uTProj * tViewModel * vec4(vPos, 1.0);
    color = vec4(uColor[gl_InstanceID].rgb, 1.0);
    if (gl_InstanceID == 0) {
        color = vec4(uColor[gl_InstanceID].rgb * (1.0 - (gl_VertexID / 200.0)), 1.0);
    }
}