#version 460

layout(vertices = 3) out;

void main()
{
    gl_TessLevelOuter[0] = 4.0;
    gl_TessLevelOuter[1] = 4.0;
    gl_TessLevelOuter[2] = 4.0;

    gl_TessLevelInner[0] = 3.0;
    gl_TessLevelInner[1] = 3.0;

    gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
}