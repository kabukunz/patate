#version 410 core

uniform float zoom;
uniform float pointRadius;
uniform float halfLineWidth;
uniform bool showWireframe;

in vec3 linearBasis;
in vec2 position;
flat in vec2 vertices[3];
flat in vec2 normEdges[3];

int minIndex(in vec3 dist)
{
    int minIdx = (dist[1] < dist[0])? 1: 0;
    if(dist[2] < dist[minIdx])
        minIdx = 2;

    return minIdx;
}

vec3 computeVertexSqrDist()
{
    return vec3(
        dot(position - vertices[0], position - vertices[0]),
        dot(position - vertices[1], position - vertices[1]),
        dot(position - vertices[2], position - vertices[2]));
}

vec3 computeEdgeDist()
{
    return vec3(
        determinant(mat2(normEdges[0], position - vertices[1])),
        determinant(mat2(normEdges[1], position - vertices[2])),
        determinant(mat2(normEdges[2], position - vertices[0])));
}

float irlerp(in vec2 vx, in vec2 v1, in vec2 v2)
{
    float alpha = acos(clamp(dot(v1, vx), -1., 1.));
    float beta = acos(clamp(dot(v1, v2), -1., 1.));
    return alpha / beta;
}

vec4 quadraticInterp(in vec4 colors[6])
{
    return
        colors[0] * linearBasis.x * (2. * linearBasis.x - 1.) +
        colors[1] * linearBasis.y * (2. * linearBasis.y - 1.) +
        colors[2] * linearBasis.z * (2. * linearBasis.z - 1.) +
        colors[3] * 4. * linearBasis.y * linearBasis.z +
        colors[4] * 4. * linearBasis.z * linearBasis.x +
        colors[5] * 4. * linearBasis.x * linearBasis.y;
}

float interpFactor(float dist, float radius)
{
    return clamp(.5 - dist*zoom + radius, 0, 1);
}

vec4 colorWithBordersAndPoints(in vec4 colorNodes[6])
{
    vec3 vertexSqrDist = computeVertexSqrDist();
    int closestVx = minIndex(vertexSqrDist);

    vec3 edgeDist = computeEdgeDist();
    int closestEdge = minIndex(edgeDist);

    vec4 color = quadraticInterp(colorNodes);

    if(showWireframe)
    {
        color = mix(color, vec4(0., 0., 0., 1.),
                    interpFactor(edgeDist[closestEdge], halfLineWidth+.5));
        color = mix(color, colorNodes[closestEdge + 3]*.5,
                    interpFactor(edgeDist[closestEdge], halfLineWidth));
        color = mix(color, vec4(0., 0., 0., 1.),
                    interpFactor(sqrt(vertexSqrDist[closestVx]), pointRadius+.5));
        color = mix(color, colorNodes[closestVx],
                    interpFactor(sqrt(vertexSqrDist[closestVx]), pointRadius));
    }

    return color;
}
