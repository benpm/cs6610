#version 460

layout(vertices = 3) out;

uniform float uTessLevel = 1.0;

void main()
{
    float outer = uTessLevel;
    float inner = uTessLevel;

    gl_TessLevelOuter[0] = outer;
    gl_TessLevelOuter[1] = outer;
    gl_TessLevelOuter[2] = outer;
    // gl_TessLevelOuter[3] = 2.0;

    gl_TessLevelInner[0] = inner;
    gl_TessLevelInner[1] = inner;

    gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
}