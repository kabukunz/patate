/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#version 410 core

uniform float zoom;
uniform float pointRadius;
uniform float halfLineWidth;
uniform bool showWireframe;
uniform vec4 wireframeColor;
uniform vec4 pointColor;

in vec3 frag_linearBasis;
in vec2 position;
flat in vec2 vertices[3];
flat in vec2 normEdges[3];


float irlerp(in vec3 vx, in vec3 v1, in vec3 v2)
{
    float alpha = acos(clamp(dot(v1, vx), -1., 1.));
    float beta = acos(clamp(dot(v1, v2), -1., 1.));
    return alpha / beta;
}


vec4 quadraticInterp(in vec4 colors[6])
{
    return
        colors[0] * frag_linearBasis.x * (2. * frag_linearBasis.x - 1.) +
        colors[1] * frag_linearBasis.y * (2. * frag_linearBasis.y - 1.) +
        colors[2] * frag_linearBasis.z * (2. * frag_linearBasis.z - 1.) +
        colors[3] * 4. * frag_linearBasis.y * frag_linearBasis.z +
        colors[4] * 4. * frag_linearBasis.z * frag_linearBasis.x +
        colors[5] * 4. * frag_linearBasis.x * frag_linearBasis.y;
}

float diffuse(in vec3 n, in vec3 l)
{
    return clamp(dot(n, l), 0., 1.);
}

vec3 linearToSrgb(in vec3 linear)
{
    vec3 srgb = linear;
    srgb[0] = (linear[0] > 0.0031308)?
              1.055 * pow(linear[0], 1./2.4):
              12.92 * linear[0];
    srgb[1] = (linear[1] > 0.0031308)?
              1.055 * pow(linear[1], 1./2.4):
              12.92 * linear[1];
    srgb[2] = (linear[2] > 0.0031308)?
              1.055 * pow(linear[2], 1./2.4):
              12.92 * linear[2];
    return srgb;
}


vec3 srgbToLinear(in vec3 srgb)
{
    vec3 linear = srgb;
    linear[0] = (linear[0] > 0.04045)?
                pow((linear[0]+0.055) / 1.055, 2.4):
                linear[0] / 12.92;
    linear[1] = (linear[1] > 0.04045)?
                pow((linear[1]+0.055) / 1.055, 2.4):
                linear[1] / 12.92;
    linear[2] = (linear[2] > 0.04045)?
                pow((linear[2]+0.055) / 1.055, 2.4):
                linear[2] / 12.92;
    return linear;
}
