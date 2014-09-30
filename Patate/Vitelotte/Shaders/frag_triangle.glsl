#version 410 core

uniform samplerBuffer nodes;
uniform float zoom;
uniform float pointRadius;
uniform float halfLineWidth;

in vec3 linearBasis;
in vec2 position;
flat in vec2 vertices[3];
flat in vec2 normEdges[3];

out vec4 out_color;

int minIndex(in vec3 dist);
vec3 computeVertexSqrDist();
vec3 computeEdgeDist();
float irlerp(in vec2 vx, in vec2 v1, in vec2 v2);
vec4 quadraticInterp(in vec4 colors[6]);
float interpFactor(float dist, float radius);
vec4 colorWithBordersAndPoints(in vec4 colorNodes[6]);

int baseVxIndex = gl_PrimitiveID * 6;
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

    out_color = colorWithBordersAndPoints(colorNodes);
}
