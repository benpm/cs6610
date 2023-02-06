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

// Blinn-phong: light direction in view-space
uniform vec3 uLightDir;

struct Material {
    vec3 diffuseColor;
    vec3 specularColor;
    vec3 ambientColor;
    float shininess;
    float specularFactor;
    float ambientFactor;
};

// SSBO for materials
layout(std430, binding = 1) buffer Materials
{
    Material uMaterial[];
};


void main() {
    Material mat = uMaterial[drawID];

    // Fragment normal
    vec3 n = normalize(normal);
    // View vector
    vec3 v = vec3(0.0, 0.0, 1.0);
    // Half-angle vector between light and view
    vec3 h = normalize(uLightDir + v);

    // Blinn shading
    vec3 diffuse = vec3(max(0.0, dot(n, uLightDir))) * mat.diffuseColor;
    vec3 specular = vec3(pow(max(0.0, dot(h, n)), mat.shininess)) * mat.specularColor;
    
    fColor = vec4(diffuse + specular * mat.specularFactor + mat.ambientColor * mat.ambientFactor, 1.0);
}
