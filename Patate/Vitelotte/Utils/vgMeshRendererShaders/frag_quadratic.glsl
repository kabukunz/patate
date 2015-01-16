/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#version 410 core

uniform samplerBuffer nodes;
uniform int baseNodeIndex;
uniform bool singularTriangles;

in vec3 linearBasis;
in vec2 position;
flat in vec2 vertices[3];
flat in vec2 normEdges[3];

out vec4 out_color;

float irlerp(in vec2 vx, in vec2 v1, in vec2 v2);
vec4 quadraticInterp(in vec4 colors[6]);

int baseVxIndex = baseNodeIndex + gl_PrimitiveID * (6 + int(singularTriangles));
int baseEdgeIndex = baseVxIndex + 3;

void main(void)
{
    vec4 colorNodes[] = vec4[6](
        texelFetch(nodes, baseVxIndex + 0),
        texelFetch(nodes, baseVxIndex + 1),
        texelFetch(nodes, baseVxIndex + 2),
        texelFetch(nodes, baseEdgeIndex + 0),
        texelFetch(nodes, baseEdgeIndex + 1),
        texelFetch(nodes, baseEdgeIndex + 2)
    );

	if(singularTriangles)
	{
		colorNodes[0] = mix(colorNodes[0],
							texelFetch(nodes, baseVxIndex + 6),
							irlerp(normalize(position - vertices[0]),
									normEdges[2], -normEdges[1]));
	}

	out_color = quadraticInterp(colorNodes);
}
