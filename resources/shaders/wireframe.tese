#version 460

layout(triangles, fractional_odd_spacing, ccw) in;

vec4 interpolate(vec4 v0, vec4 v1, vec4 v2) {
    return gl_TessCoord.x * v0 + gl_TessCoord.y * v1 + gl_TessCoord.z * v2;
}

// Bilinear interpolate using gl_TessCoord.xy
// vec4 interpolate(vec4 v0, vec4 v1, vec4 v2, vec4 v3) {
//     vec4 v01 = mix(v0, v1, gl_TessCoord.x);
//     vec4 v23 = mix(v2, v3, gl_TessCoord.x);
//     return mix(v01, v23, gl_TessCoord.y);
// }

void main() {
    gl_Position = interpolate(
        gl_in[0].gl_Position,
        gl_in[1].gl_Position,
        gl_in[2].gl_Position);
}