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

uniform mat4 uTView;

struct Material {
    vec3 diffuseColor;
    vec3 specularColor;
    vec3 ambientColor;
    float shininess;
    float specularFactor;
    float ambientFactor;
};

struct Light {
    vec3 position;      // Light position in view space
    vec3 color;         // Light color
    float intensity;    // Light intensity
};

// SSBO for materials
layout(std430, binding = 1) buffer Materials
{
    Material uMaterial[];
};

// SSBO for lights
layout(std430, binding = 2) buffer Lights
{
    uint uLightCount;
    Light uLight[];
};


void main() {
    Material mat = uMaterial[drawID];
    // Fragment normal
    vec3 n = normalize(normal);

    // Accumulate lights
    vec3 C = mat.ambientColor * mat.ambientFactor;
    for (uint i = 0; i < uLightCount; i++) {
        // Vector to light
        vec3 lightVec = (uTView * vec4(uLight[i].position, 1.0)).xyz - position;
        // Direction to light
        vec3 lightDir = normalize(lightVec);
        // Half-angle vector between light and view
        vec3 h = normalize(lightDir + vec3(0.0, 0.0, 1.0));

        // Blinn shading
        vec3 diffuse = vec3(max(0.0, dot(n, lightDir))) * mat.diffuseColor;
        vec3 specular = vec3(pow(max(0.0, dot(h, n)), mat.shininess)) * mat.specularColor;
        
        C += pow((1.0/length(lightVec)) * 10.0, 2.0) * (diffuse + specular * mat.specularFactor);
    }

    fColor = vec4(C, 1.0);
}
