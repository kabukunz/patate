/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#version 410 core

uniform mat4 viewMatrix;

in vec4 vx_position;

out vec2 position_obj;

void main(void)
{
    gl_Position = viewMatrix * vx_position;
    position_obj = vx_position.xy;
}
