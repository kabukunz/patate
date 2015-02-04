/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#version 410 core

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

in vec2 position_obj[];

out int gl_PrimitiveID;
out vec3 linearBasis;
out vec2 position;
flat out vec2 vertices[3];
flat out vec2 normEdges[3];

const vec3 basis[3] = vec3[3](
    vec3(1, 0, 0),
    vec3(0, 1, 0),
    vec3(0, 0, 1)
);

void main()
{
    for(int i=0; i<3; ++i)
    {
        gl_Position = gl_in[i].gl_Position;
        gl_PrimitiveID = gl_PrimitiveIDIn;
        linearBasis = basis[i];
        position = position_obj[i];//gl_in[i].gl_Position.xy;
        for(int j=0; j<3; ++j)
        {
            vertices[j] = position_obj[j];//gl_in[j].gl_Position.xy;
            normEdges[j] = normalize(vertices[(j+2)%3] - vertices[(j+1)%3]);
        }
        EmitVertex();
    }
}
