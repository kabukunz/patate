uniform highp float lineWidth;
uniform lowp vec4 wireframeColor;
uniform highp float zoom;

varying highp vec2 position;
varying highp vec3 basis;
varying highp vec4 edges;


highp float interpFactor(highp float dist, highp float radius)
{
    return clamp(.5 + radius - dist*zoom, 0., 1.);
}

void main(void)
{
    highp vec2 v0 = edges.xy;
    highp vec2 v1 = edges.zw;
    highp vec2 v2 = -(v0 + v1);

    highp float _2area = v0.x * v1.y - v0.y * v1.x;
    highp vec3 edgeDist = basis * _2area / vec3(length(v0), length(v1), length(v2));

    highp float dist = 0.;
    if(edgeDist.x < edgeDist.y && edgeDist.x < edgeDist.z)
        dist = edgeDist.x;
    else if(edgeDist.y < edgeDist.x && edgeDist.y < edgeDist.z)
        dist = edgeDist.y;
    else //if(edgeDist.z < edgeDist.x && edgeDist.z < edgeDist.y)
        dist = edgeDist.z;

    highp float intensity = interpFactor(dist, lineWidth/2.);
    if(intensity < 0.001)
        discard;

    gl_FragColor = vec4(wireframeColor.rgb, wireframeColor.a * intensity);
}
