#version 460

layout(triangles, fractional_odd_spacing, ccw) in;

in VertexData {
    vec2 uv;
    vec3 normal;
    uint matID;
    uint drawID;
} data_in[];

out VertexData {
    vec2 uv;
    vec3 normal;
    vec3 position;
    vec3 wnormal;
    vec3 wposition;
    flat uint matID;
    flat uint drawID;
} data_out;

uniform mat4 uTProj;
uniform mat4 uTView;

layout(std430, binding = 0) buffer ModelTransforms {
    mat4 tModel[];
};

uniform sampler2D uTex[16];

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

vec4 interpolate(vec4 v0, vec4 v1, vec4 v2) {
    return gl_TessCoord.x * v0 + gl_TessCoord.y * v1 + gl_TessCoord.z * v2;
}

vec3 interpolate(vec3 v0, vec3 v1, vec3 v2) {
    return gl_TessCoord.x * v0 + gl_TessCoord.y * v1 + gl_TessCoord.z * v2;
}

vec2 interpolate(vec2 v0, vec2 v1, vec2 v2) {
    return gl_TessCoord.x * v0 + gl_TessCoord.y * v1 + gl_TessCoord.z * v2;
}

void main() {
    Material mat = uMaterial[data_in[0].matID];
    mat4 tmodel = tModel[data_in[0].drawID];

    data_out.drawID = data_in[0].drawID;
    data_out.matID = data_in[0].matID;
    data_out.uv = interpolate(data_in[0].uv, data_in[1].uv, data_in[2].uv);
    vec3 normal = interpolate(data_in[0].normal, data_in[1].normal, data_in[2].normal);
    data_out.normal = normalize((uTView * tmodel * vec4(normal, 0.0)).xyz);
    
    // Displaced position in world space
    float disp = 0.0;
    if (mat.displacementTexID >= 0) {
        disp = texture(uTex[mat.displacementTexID], data_out.uv).r * 0.1;
    }
    vec4 v = interpolate(
        gl_in[0].gl_Position,
        gl_in[1].gl_Position,
        gl_in[2].gl_Position) + vec4(disp * data_out.normal, 0.0);

    vec4 wposition = tmodel * v;
    vec4 vposition = uTView * wposition;
    data_out.wposition = wposition.xyz;
    data_out.position = vposition.xyz;
    data_out.wnormal = (tmodel * vec4(normal, 0.0)).xyz;

    gl_Position = uTProj * vposition;
    
}