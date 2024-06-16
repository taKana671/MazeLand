#version 300 es
precision highp float;

uniform float tex_ScaleFactor0;
uniform float tex_ScaleFactor1;
uniform float tex_ScaleFactor2;
uniform float tex_ScaleFactor3;

uniform sampler2D p3d_Texture0;
uniform sampler2D p3d_Texture1;
uniform sampler2D p3d_Texture2;
uniform sampler2D p3d_Texture3;

in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in vec2 texcoord3;

in vec4 vertex;
out vec4 fragColor;

float computeWeight(float min_z, float max_z, vec4 vertex){
    float region = max_z - min_z;
    return max(0.0, (region - abs(vertex.z - max_z)) / region);
}

void main() {
    vec4 tex0 = texture(p3d_Texture0, texcoord0.st * tex_ScaleFactor0).rgba;
    vec4 tex1 = texture(p3d_Texture1, texcoord1.st * tex_ScaleFactor1).rgba;
    vec4 tex2 = texture(p3d_Texture2, texcoord2.st * tex_ScaleFactor2).rgba;
    vec4 tex3 = texture(p3d_Texture3, texcoord3.st * tex_ScaleFactor3).rgba;

    float scale = 500.0;
    float min_z = 0.0;
    float max_z = 0.0;
   
    // tex0
    min_z = -100.0/scale;
    max_z = 0.0/scale;
    float w0 = computeWeight(min_z, max_z, vertex);
 
    // tex1
    min_z = 1.0/scale;
    max_z = 100.0/scale;
    float w1 = computeWeight(min_z, max_z, vertex);

    // tex2
    min_z = 101.0/scale;
    max_z = 300.0/scale;
    float w2 = computeWeight(min_z, max_z, vertex);

    // tex3
    min_z = 301.0/scale;
    max_z = 500.0/scale;
    float w3 = computeWeight(min_z, max_z, vertex);

    fragColor = tex0 * w0 + tex1 * w1 + tex2 * w2 + tex3 * w3;
}

