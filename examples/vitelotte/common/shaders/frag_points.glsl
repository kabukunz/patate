#version 410 core

flat in vec4 color;
in vec2 texCoord;
flat in float radius;

out vec4 out_color;

void main(void)
{
	float alpha = radius - length(texCoord);

	if(alpha <= 0)
		discard;
	alpha = clamp(alpha, 0, 1);
	out_color = mix(vec4(color.rgb, 0.), color, alpha);
}
