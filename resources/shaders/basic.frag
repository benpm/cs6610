#version 460

const float pi = 3.1415926535897932384626433832795;

// View space normal
layout(location = 0) in vec3 normal;
// View space position
layout(location = 1) in vec3 position;
// Draw ID
layout(location = 2) flat in uint drawID;
// UV coordinates for texture sampling
layout(location = 3) in vec2 uv;
// Material ID
layout(location = 4) flat in uint matID;
// World space normal
layout(location = 5) in vec3 wnormal;
// World space position
layout(location = 6) in vec3 wposition;

// Fragment color
out vec4 fColor;

uniform uint nLights;
uniform sampler2D uTex[16];
uniform samplerCube uEnvTex;
uniform samplerCubeShadow uShadowMap;
uniform sampler2DShadow uSpotShadowMap;

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
    int reflectionTexID;
    int flatReflectionTexID;
};

const uint LIGHT_POINT = 0;
const uint LIGHT_DIRECTIONAL = 1;
const uint LIGHT_SPOT = 2;

struct Light {
    vec3 position;      // Light position in world space
    vec3 color;         // Light color
    vec3 direction;     // Light direction
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

uniform mat4 uTProj;
uniform mat4 uTView;
uniform vec3 uCamPos;
uniform vec2 uViewport;
uniform vec3 uLightPos;
uniform vec3 uSpotLightPos;
uniform float uFarPlane;
uniform mat4 uTSpotLight;

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
        vec3 viewLightPos = (uTView * vec4(uLight[i].position, 1.0)).xyz;
        // Direction to light
        vec3 lightVec = viewLightPos - position;
        vec3 lightDir = normalize(lightVec);
        if (uLight[i].type == LIGHT_DIRECTIONAL) {
            lightDir = (uTView * vec4(uLight[i].direction, 0.0)).xyz;
        }

        // Blinn shading
        vec3 h = normalize(lightDir + vec3(0.0, 0.0, 1.0));
        vec3 diffuse = max(0.0, dot(n, lightDir)) * diffuseColor;
        vec3 specular = pow(max(0.0, dot(h, n)), max(0.1, mat.shininess)) * specularColor;

        // Light attenuation
        float attenuation = uLight[i].intensity;
        if (uLight[i].type == LIGHT_POINT) {
            attenuation = pow((1.0/length(lightVec)) * uLight[i].intensity, 2.0);
        } else if (uLight[i].type == LIGHT_SPOT) {
            float theta = dot(
                normalize(uLight[i].position - wposition),
                normalize(-uLight[i].direction));
            if (theta < cos(pi / 4.0)) {
                attenuation = 0.0;
            } else {
                attenuation = pow((1.0/length(lightVec)) * uLight[i].intensity, 2.0);
            }
        }

        if (uLight[i].type == LIGHT_POINT) {
            // Shadow mapping for point light
            vec3 fragRelToLight = wposition - uLightPos;
            attenuation *= texture(uShadowMap, vec4(
                fragRelToLight, length(fragRelToLight) / uFarPlane - 0.01));
        } else if (uLight[i].type == LIGHT_SPOT) {
            // Shadow mapping for spotlight
            vec4 v = uTSpotLight * vec4(wposition, 1.0);
            attenuation *= texture(uSpotShadowMap, vec3(
                v.xy / (2.0 * v.w) + 0.5,
                min(1.0, length(v.xyz / uFarPlane) - 0.01)));
        }
        
        C += attenuation * (diffuse + specular * mat.specularFactor);
    }

    // Environment mapping / reflection
    if (mat.reflectionTexID >= 0) {
        vec3 reflectionTex = texture(uEnvTex,
            reflect(wposition - uCamPos, normalize(wnormal))).rgb;
        C = reflectionTex;
    }
    if (mat.flatReflectionTexID >= 0) {
        vec2 ts = (gl_FragCoord.xy / uViewport);
        vec3 reflectionTex = texture(uTex[mat.flatReflectionTexID],
            vec2(ts.x, 1.0 - ts.y)).rgb;
        C = reflectionTex;
    }

    C = mix(C, mat.emissionColor, mat.emissionFactor);
    
    fColor = vec4(C, 1.0);
}
