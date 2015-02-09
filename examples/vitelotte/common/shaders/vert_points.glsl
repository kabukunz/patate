#version 410 core

uniform mat4 viewMatrix;

in vec4 vx_position;
in vec4 vx_color;

out vec4 point_color;
out float point_radius;

void main(void)
{
	gl_Position = viewMatrix * vec4(vx_position.xyz, 1.);

	point_color = vx_color;
	point_radius = vx_position.w;
}
