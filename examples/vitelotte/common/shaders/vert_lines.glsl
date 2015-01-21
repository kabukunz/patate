#version 410 core

uniform mat4 viewMatrix;

in vec4 vx_position;
in vec4 vx_color;

out vec4 line_position;
out vec4 line_color;

void main(void)
{
	gl_Position = viewMatrix * vec4(vx_position.xyz, 1.);

	line_position = vx_position;
	line_color = vx_color;
}
