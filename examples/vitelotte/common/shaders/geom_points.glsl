#version 410 core

layout(points) in;
layout(triangle_strip, max_vertices = 4) out;

uniform vec2 viewportSize;

in vec4 point_color[];
in float point_radius[];

flat out vec4 color;
out vec2 texCoord;
flat out float radius;

void main()
{
	vec2 sp = gl_in[0].gl_Position.xy / gl_in[0].gl_Position.w;

	float pixelRadius = point_radius[0] + .5001;
	vec2 r = pixelRadius * 2. / viewportSize;

	color = point_color[0];
	radius = pixelRadius;

	gl_Position = vec4(
			gl_in[0].gl_Position.xy + vec2(-r.x,  r.y) * gl_in[0].gl_Position.w,
			gl_in[0].gl_Position.zw);
	texCoord = vec2(-pixelRadius,  pixelRadius);
	EmitVertex();

	gl_Position = vec4(
			gl_in[0].gl_Position.xy + vec2(-r.x, -r.y) * gl_in[0].gl_Position.w,
			gl_in[0].gl_Position.zw);
	texCoord = vec2(-pixelRadius, -pixelRadius);
	EmitVertex();

	gl_Position = vec4(
			gl_in[0].gl_Position.xy + vec2( r.x,  r.y) * gl_in[0].gl_Position.w,
			gl_in[0].gl_Position.zw);
	texCoord = vec2( pixelRadius,  pixelRadius);
	EmitVertex();

	gl_Position = vec4(
			gl_in[0].gl_Position.xy + vec2( r.x, -r.y) * gl_in[0].gl_Position.w,
			gl_in[0].gl_Position.zw);
	texCoord = vec2( pixelRadius, -pixelRadius);
	EmitVertex();
}
