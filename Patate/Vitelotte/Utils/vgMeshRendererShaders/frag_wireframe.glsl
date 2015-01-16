/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#version 410 core

uniform samplerBuffer nodes;
uniform float lineWidth;
uniform vec4 wireframeColor;

in vec3 linearBasis;
in vec2 position;
flat in vec2 vertices[3];
flat in vec2 normEdges[3];

out vec4 out_color;

vec3 computeEdgeDist();
int minIndex(in vec3 dist);
float interpFactor(float dist, float radius);

void main(void)
{
	vec3 edgeDist = computeEdgeDist();
	int closestEdge = minIndex(edgeDist);
	float intensity = interpFactor(edgeDist[closestEdge], lineWidth/2.);
	if(intensity < 0.001)
		discard;

	out_color = vec4(wireframeColor.rgb, wireframeColor.a * intensity);
}
