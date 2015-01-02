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

int baseVxIndex = baseNodeIndex + gl_PrimitiveID * (3 + int(singularTriangles));

vec4 linearInterp(in vec4 colors[3])
{
    return
        colors[0] * linearBasis.x +
        colors[1] * linearBasis.y +
        colors[2] * linearBasis.z;
}

void main(void)
{
    vec4 colorNodes[] = vec4[3](
        texelFetch(nodes, baseVxIndex + 0),
        texelFetch(nodes, baseVxIndex + 1),
        texelFetch(nodes, baseVxIndex + 2)
    );

	if(singularTriangles)
	{
		colorNodes[0] = mix(colorNodes[0],
                            texelFetch(nodes, baseVxIndex + 3),
							irlerp(normalize(position - vertices[0]),
									normEdges[2], -normEdges[1]));
	}

    out_color = linearInterp(colorNodes);
}
