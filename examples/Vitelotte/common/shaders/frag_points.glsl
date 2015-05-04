/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

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
