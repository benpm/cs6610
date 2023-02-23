#version 460

const float pi = 3.1415926535897932384626433832795;

// Vertex color
layout(location = 0) in vec4 color;
// View space normal
layout(location = 1) in vec3 normal;
// View space position
layout(location = 2) in vec3 position;
// Draw ID
layout(location = 3) flat in uint drawID;
// UV coordinates for texture sampling
layout(location = 4) in vec2 uv;
// Material ID
layout(location = 5) flat in uint matID;

// Fragment color
out vec4 fColor;

uniform uint nLights;
uniform sampler2D uTex[32];
uniform samplerCube uCubeTex[32];

struct Material {
    vec3 diffuseColor;
    vec3 specularColor;
    vec3 ambientColor;
    float shininess;
    float specularFactor;
    float ambientFactor;
    int diffuseTexID;
    int specularTexID;
    int reflectionTexID;
};

const uint lightPoint = 0;
const uint lightDirectional = 1;

struct Light {
    vec3 position;      // Light position in view space (direction if directional)
    vec3 color;         // Light color
    float intensity;    // Light intensity
    uint type;          // Light type (point or directional)
};

// SSBO for materials
layout(std430, binding = 1) buffer Materials
{
    Material uMaterial[];
};

// SSBO for lights
layout(std430, binding = 2) buffer Lights
{
    Light uLight[];
};


void main() {
    Material mat = uMaterial[matID];
    // Fragment normal
    vec3 n = normalize(normal);

    vec3 diffuseTex = mat.diffuseColor;
    if (mat.diffuseTexID >= 0) {
        diffuseTex = texture(uTex[mat.diffuseTexID], uv).rgb;
    }
    vec3 specularTex = mat.specularColor;
    if (mat.specularTexID >= 0) {
        specularTex = texture(uTex[mat.specularTexID], uv).rgb;
    }
    vec3 diffuseColor = diffuseTex;
    vec3 specularColor = specularTex;

    // Accumulate lights
    vec3 C = mat.ambientColor * mat.ambientFactor;
    for (uint i = 0; i < nLights; i++) {
        // Vector to light
        vec3 lightVec = uLight[i].position - position;
        if (uLight[i].type == lightDirectional) {
            lightVec = uLight[i].position;
        }
        // Direction to light
        vec3 lightDir = normalize(lightVec);
        // Half-angle vector between light and view
        vec3 h = normalize(lightDir + vec3(0.0, 0.0, 1.0));

        // Blinn shading
        vec3 diffuse = max(0.0, dot(n, lightDir)) * diffuseColor;
        vec3 specular = pow(max(0.0, dot(h, n)), max(0.1, mat.shininess)) * specularColor;

        float attenuation = uLight[i].intensity;
        if (uLight[i].type == lightPoint) {
            attenuation = pow((1.0/length(lightVec)) * uLight[i].intensity, 2.0);
        }
        
        C += attenuation * (diffuse + specular * mat.specularFactor);

        // Environment mapping / reflection
        if (mat.reflectionTexID >= 0) {
            vec3 reflection = reflect(lightDir, n);
            vec3 reflectionTex = texture(uCubeTex[mat.reflectionTexID], reflection).rgb;
            C = reflectionTex;
        }
    }

    // Debug: color all invalid values as magenta
    if (any(isnan(C) || isinf(C))) {
        C = vec3(1.0, 0.0, 1.0);
    }
    
    fColor = vec4(C, 1.0);
}
