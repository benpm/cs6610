#version 460

const float pi = 3.1415926535897932384626433832795;

// Vertex color
layout(location = 0) in vec4 color;
// View space normal
layout(location = 1) in vec3 normal;
// View space position
layout(location = 2) in vec3 position;

// Fragment color
out vec4 fColor;

// Blinn-phong: light direction in view-space
uniform vec3 uLightDir;


void main() {
    // Fragment normal
    vec3 n = normalize(normal);
    // Light reflection vector
    vec3 r = normalize(reflect(-uLightDir, n));
    // View vector
    vec3 v = vec3(0.0, 0.0, 1.0);
    // Half-angle vector between light and view
    vec3 h = normalize(uLightDir + v);

    // Blinn shading
    vec3 diffuse = vec3(max(0.0, dot(n, uLightDir))) * vec3(1.0, 0.31, 0.95);
    vec3 specular = vec3(pow(max(0.0, dot(h, n)), 35.0)) * vec3(1.0, 0.31, 0.95) * 3.0;
    vec3 ambient = vec3(0.15, 0.05, 0.02);
    
    fColor = vec4(diffuse + specular, 1.0);
}
