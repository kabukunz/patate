/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

uniform mat4 viewMatrix;

attribute highp vec4 vx_position;
attribute highp vec3 vx_basis;
attribute highp vec4 vx_edges;
attribute mediump vec2 vx_baseNodeCoord;

varying highp vec3 basis;
varying highp vec4 edges;
varying mediump vec2 baseNodeCoord;

void main(void)
{
    gl_Position = viewMatrix * vx_position;
    basis = vx_basis;
    edges = vx_edges;
    baseNodeCoord = vx_baseNodeCoord;
}