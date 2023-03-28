#version 460

layout(location = 0) in vec3 vPos;
layout(location = 1) in vec3 vNormal;
layout(location = 2) in vec3 vUV;
layout(location = 3) in uint vMatID;


out VertexData {
    vec2 uv;
    vec3 normal;
    uint matID;
    uint drawID;
} data_out;

void main()
{
    gl_Position = vec4(vPos, 1.0);
    data_out.uv = vUV.xy;
    data_out.normal = vNormal;
    data_out.matID = vMatID;
    data_out.drawID = gl_DrawID;
}