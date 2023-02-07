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

// Fragment color
out vec4 fColor;

uniform uint nLights;

struct Material {
    vec3 diffuseColor;
    vec3 specularColor;
    vec3 ambientColor;
    float shininess;
    float specularFactor;
    float ambientFactor;
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
    Material mat = uMaterial[drawID];
    // Fragment normal
    vec3 n = normalize(normal);

    // Accumulate lights
    vec3 C = mat.ambientColor * mat.ambientFactor;
    for (uint i = 0; i < nLights; i++) {
        // Vector to light
        vec3 lightVec = uLight[i].position - position;
        if (uLight[i].type == lightDirectional) {
            lightVec = -uLight[i].position;
        }
        // Direction to light
        vec3 lightDir = normalize(lightVec);
        // Half-angle vector between light and view
        vec3 h = normalize(lightDir + vec3(0.0, 0.0, 1.0));

        // Blinn shading
        vec3 diffuse = vec3(max(0.0, dot(n, lightDir))) * mat.diffuseColor;
        vec3 specular = vec3(pow(max(0.0, dot(h, n)), mat.shininess)) * mat.specularColor;

        float attenuation = uLight[i].intensity;
        if (uLight[i].type == lightPoint) {
            attenuation = pow((1.0/length(lightVec)) * uLight[i].intensity, 2.0);
        }
        
        C += attenuation * (diffuse + specular * mat.specularFactor);
    }

    fColor = vec4(C, 1.0);
}
