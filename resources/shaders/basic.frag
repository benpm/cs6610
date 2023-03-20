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
uniform samplerCubeArrayShadow uCubeShadowMaps;
uniform sampler2DArrayShadow u2DShadowMaps;

uniform mat4 uTProj;
uniform mat4 uTView;
uniform vec3 uCamPos;
uniform vec2 uViewport;

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
    float range;        // Light range
    float spotAngle;    // Spotlight angle
    float far;          // Spotlight far plane
    uint type;          // Light type (point or directional)
    int shadowMapLayer; // Shadow map layer ID (-1 if none)
    mat4 transform;     // Light-space transform
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

// From Cem Yuksel "Point Light Attenuation Without Singularity"
float lightAttenuate(float d, float r) {
    return (2.0 / (r*r)) * (1.0 - d / sqrt(d*d + r*r));
}

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
        const vec3 viewLightPos = (uTView * vec4(uLight[i].position, 1.0)).xyz;
        const vec3 lightVec = viewLightPos - position;
        vec3 lightDir = normalize(lightVec);

        // Light attenuation
        float attenuation = 0.0;
        switch (uLight[i].type) {
            case LIGHT_POINT: {
                attenuation = lightAttenuate(length(lightVec), uLight[i].range) * uLight[i].intensity;
                if (uLight[i].shadowMapLayer >= 0) {
                    // Shadow mapping for point light
                    vec3 wLightVec = wposition - uLight[i].position;
                    attenuation *= texture(uCubeShadowMaps, vec4(
                        wLightVec, uLight[i].shadowMapLayer), length(wLightVec) / uLight[i].far - 0.01);
                }
                break; }
            case LIGHT_SPOT: {
                float theta = dot(
                    normalize(uLight[i].position - wposition),
                    normalize(-uLight[i].direction));
                if (theta >= uLight[i].spotAngle) {
                    attenuation = lightAttenuate(length(lightVec), uLight[i].range) * uLight[i].intensity;
                    // Shadow mapping for spotlight
                    if (uLight[i].shadowMapLayer >= 0) {
                        vec4 wLightVec = uLight[i].transform * vec4(wposition, 1.0);
                        attenuation *= texture(u2DShadowMaps, vec4(
                            wLightVec.xy / (2.0 * wLightVec.w) + 0.5,
                            min(1.0, length(wLightVec.xyz / uLight[i].far) - 0.01), uLight[i].shadowMapLayer));
                    }
                }
                break; }
            case LIGHT_DIRECTIONAL: {
                lightDir = (uTView * vec4(uLight[i].direction, 0.0)).xyz;
                attenuation = uLight[i].intensity;
                break; }
        }

        // Blinn shading
        const vec3 h = normalize(lightDir + vec3(0.0, 0.0, 1.0));
        const vec3 diffuse = max(0.0, dot(n, lightDir)) * diffuseColor;
        const vec3 specular = pow(max(0.0, dot(h, n)), max(0.1, mat.shininess)) * specularColor;
        
        C += attenuation * uLight[i].color * (diffuse + specular * mat.specularFactor);
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
