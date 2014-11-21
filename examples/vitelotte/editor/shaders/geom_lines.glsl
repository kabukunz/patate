#version 410 core

layout(lines) in;
layout(triangle_strip, max_vertices = 4) out;

uniform vec2 viewportSize;

in vec4 line_position[];
in vec4 line_color[];

flat out vec4 color[2];
out vec2 texCoord;
flat out vec3 line;

void main()
{
	vec2 sp0 = gl_in[0].gl_Position.xy * viewportSize / (gl_in[0].gl_Position.w * 2.);
	vec2 sp1 = gl_in[1].gl_Position.xy * viewportSize / (gl_in[1].gl_Position.w * 2.);

	float len = length(sp1 - sp0);
	vec2 v = (sp1 - sp0) / len;
	vec2 n = vec2(-v.y, v.x);

	// w parameter of position encode the line width
	float r = max(line_position[0].w, line_position[1].w) / 2. + .5001;

	color[0] = line_color[0];
	color[1] = line_color[1];
	line = vec3(line_position[0].w / 2. + .5001, line_position[1].w / 2. + .5001, len);

	gl_Position = vec4(
			(sp0 + (-v + n)*r) * gl_in[0].gl_Position.w * 2. / viewportSize,
			gl_in[0].gl_Position.zw);
	texCoord = vec2(-r, r);
	EmitVertex();

	gl_Position = vec4(
			(sp0 + (-v - n)*r) * gl_in[0].gl_Position.w * 2. / viewportSize,
			gl_in[0].gl_Position.zw);
	texCoord = vec2(-r, -r);
	EmitVertex();

	gl_Position = vec4(
			(sp1 + ( v + n)*r) * gl_in[1].gl_Position.w * 2. / viewportSize,
			gl_in[1].gl_Position.zw);
	texCoord = vec2(len + r, r);
	EmitVertex();

	gl_Position = vec4(
			(sp1 + ( v - n)*r) * gl_in[1].gl_Position.w * 2. / viewportSize,
			gl_in[1].gl_Position.zw);
	texCoord = vec2(len + r, -r);
	EmitVertex();
}
