#version 460

layout(vertices = 3) out;

uniform float uTessLevel = 1.0;

in VertexData {
    vec2 uv;
    vec3 normal;
    uint matID;
    uint drawID;
} data_in[];

out VertexData {
    vec2 uv;
    vec3 normal;
    uint matID;
    uint drawID;
} data_out[];

struct Material {
    vec3 diffuseColor;
    vec3 specularColor;
    vec3 ambientColor;
    vec3 emissionColor;
    float shininess;
    float specularFactor;
    float ambientFactor;
    float emissionFactor;
    int diffuseTexID;
    int specularTexID;
    int reflectionLayer;
    int flatReflectionTexID;
    int normalTexID;
    int displacementTexID;
};

layout(std430, binding = 1) buffer Materials {
    Material uMaterial[];
};

void main()
{
    if (gl_InvocationID == 0) {
        float tessLvl = 1.0;
        if (uMaterial[data_in[gl_InvocationID].matID].displacementTexID >= 0) {
            tessLvl = uTessLevel;
        }
        float outer = tessLvl;
        float inner = tessLvl;

        gl_TessLevelOuter[0] = outer;
        gl_TessLevelOuter[1] = outer;
        gl_TessLevelOuter[2] = outer;

        gl_TessLevelInner[0] = inner;
        gl_TessLevelInner[1] = inner;
    }

    gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;

    data_out[gl_InvocationID].uv = data_in[gl_InvocationID].uv;
    data_out[gl_InvocationID].normal = data_in[gl_InvocationID].normal;
    data_out[gl_InvocationID].matID = data_in[gl_InvocationID].matID;
    data_out[gl_InvocationID].drawID = data_in[gl_InvocationID].drawID;
}