uniform sampler2D nodes;
uniform int baseNodeIndex;
uniform bool singularTriangles;
uniform int nodesTextureSize;

varying highp vec2 position;
varying highp vec3 basis;
varying highp vec4 edges;
varying mediump vec2 baseNodeCoord;


highp float irlerp(in highp vec2 vx, in highp vec2 v1, in highp vec2 v2)
{
    highp float alpha = acos(clamp(dot(v1, vx), -1., 1.));
    highp float beta = acos(clamp(dot(v1, v2), -1., 1.));
    return alpha / beta;
}

lowp vec4 quadraticInterp(in lowp vec4 colors[6])
{
    return
        colors[0] * basis.x * (2. * basis.x - 1.) +
        colors[1] * basis.y * (2. * basis.y - 1.) +
        colors[2] * basis.z * (2. * basis.z - 1.) +
        colors[3] * 4. * basis.y * basis.z +
        colors[4] * 4. * basis.z * basis.x +
        colors[5] * 4. * basis.x * basis.y;
}

void main(void)
{
    mediump float nodeAdvance = 1. / float(nodesTextureSize);
    lowp vec4 colorNodes[6];
    colorNodes[0] = texture2D(nodes, baseNodeCoord + vec2(0. * nodeAdvance, 0.));
    colorNodes[1] = texture2D(nodes, baseNodeCoord + vec2(1. * nodeAdvance, 0.));
    colorNodes[2] = texture2D(nodes, baseNodeCoord + vec2(2. * nodeAdvance, 0.));
    colorNodes[3] = texture2D(nodes, baseNodeCoord + vec2(3. * nodeAdvance, 0.));
    colorNodes[4] = texture2D(nodes, baseNodeCoord + vec2(4. * nodeAdvance, 0.));
    colorNodes[5] = texture2D(nodes, baseNodeCoord + vec2(5. * nodeAdvance, 0.));

    if(singularTriangles)
    {
        highp vec2 p1 = edges.xy;
        highp vec2 p2 = edges.zw + p1;
        highp vec2 pos = basis[1] * p1 + basis[2] * p2;
        colorNodes[0] = mix(colorNodes[0],
            texture2D(nodes, baseNodeCoord + vec2(6. * nodeAdvance, 0.)),
            irlerp(normalize(pos), normalize(p1), normalize(p2)));
    }

    gl_FragColor = quadraticInterp(colorNodes);
}
