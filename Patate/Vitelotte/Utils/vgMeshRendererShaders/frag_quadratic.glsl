/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#version 410 core

uniform samplerBuffer nodes;
uniform int baseNodeIndex;
uniform bool singularTriangles;

flat in int frag_index;
in vec3 frag_linearBasis;
in vec3 frag_position_obj;
flat in vec3 frag_vertices_obj[3];
flat in vec3 frag_normEdges_obj[3];

out vec4 out_color;

float irlerp(in vec3 vx, in vec3 v1, in vec3 v2);
vec4 quadraticInterp(in vec4 colors[6]);

int baseVxIndex = baseNodeIndex + frag_index * (6 + int(singularTriangles));
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
        colorNodes[0] = mix(
            colorNodes[0],
            texelFetch(nodes, baseVxIndex + 6),
            irlerp(normalize(frag_position_obj - frag_vertices_obj[0]),
                   frag_normEdges_obj[2], -frag_normEdges_obj[1]));
    }

    out_color = quadraticInterp(colorNodes);
}
