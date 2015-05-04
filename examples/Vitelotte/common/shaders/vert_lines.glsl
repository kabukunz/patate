/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

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
