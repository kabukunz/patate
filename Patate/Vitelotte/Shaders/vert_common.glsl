#version 410 core

uniform mat4 viewMatrix;

in vec4 vx_position;

out vec2 position_obj;

void main(void)
{
    gl_Position = viewMatrix * vx_position;
    position_obj = vx_position.xy;
}
