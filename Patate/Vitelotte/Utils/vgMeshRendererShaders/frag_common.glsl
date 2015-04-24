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
