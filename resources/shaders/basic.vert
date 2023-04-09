#version 460

layout(location = 0) in vec3 vPos;
layout(location = 1) in vec3 vNormal;
layout(location = 2) in vec3 vUV;
layout(location = 3) in uint vMatID;

out VertexData {
    vec2 uv;
    vec3 normal;
    vec3 position;
    vec3 wnormal;
    vec3 wposition;
    flat uint matID;
    flat uint drawID;
} v_out;

uniform mat4 uTProj;
uniform mat4 uTView;

layout(std430, binding = 0) buffer ModelTransforms {
    mat4 tModel[];
};

void main()
{
    const mat4 tViewModel = uTView * tModel[gl_DrawID];
    gl_Position = uTProj * tViewModel * vec4(vPos, 1.0);
    v_out.normal = (tViewModel * vec4(vNormal, 0.0)).xyz;
    v_out.wnormal = (tModel[gl_DrawID] * vec4(vNormal, 0.0)).xyz;
    v_out.position = (tViewModel * vec4(vPos, 1.0)).xyz;
    v_out.wposition = (tModel[gl_DrawID] * vec4(vPos, 1.0)).xyz;
    v_out.uv = vUV.xy;
    v_out.drawID = gl_DrawID;
    v_out.matID = vMatID;
}