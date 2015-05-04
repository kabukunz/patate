#version 410 core

flat in vec4 color[2];
in vec2 texCoord;
flat in vec3 line;

out vec4 out_color;

void main(void)
{
	float r0 = line[0];
	float r1 = line[1];
	float len = line[2];
	float alpha;
    if(texCoord.x < 0.)
		alpha = r0 - length(texCoord);
	else if(texCoord.x > len)
        alpha = r1 - length(texCoord - vec2(len, 0.));
	else
        alpha = mix(r0, r1, texCoord.x / len) - abs(texCoord.y);

    if(alpha <= 0.)
		discard;
    alpha = clamp(alpha, 0., 1.);
    vec4 c = mix(color[0], color[1], texCoord.x / len);
    out_color = mix(vec4(c.rgb, 0.), c, alpha);
    //out_color = c;
	//out_color = vec4(vec3(fract((texCoord.x)/r0)), 1.);
    //out_color = vec4(texCoord, 0., 1.);
}
