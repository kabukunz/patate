// Generated by shader2cpp.
// 2014-11-14T17:07:59.526825


namespace Vitelotte
{
namespace shader
{

static const char* frag_common_glsl =
	"#version 410 core\n"
	"\n"
	"uniform float zoom;\n"
	"uniform float pointRadius;\n"
	"uniform float halfLineWidth;\n"
	"uniform bool showWireframe;\n"
	"uniform vec4 wireframeColor;\n"
	"uniform vec4 pointColor;\n"
	"\n"
	"in vec3 linearBasis;\n"
	"in vec2 position;\n"
	"flat in vec2 vertices[3];\n"
	"flat in vec2 normEdges[3];\n"
	"\n"
	"int minIndex(in vec3 dist)\n"
	"{\n"
	"    int minIdx = (dist[1] < dist[0])? 1: 0;\n"
	"    if(dist[2] < dist[minIdx])\n"
	"        minIdx = 2;\n"
	"\n"
	"    return minIdx;\n"
	"}\n"
	"\n"
	"vec3 computeVertexSqrDist()\n"
	"{\n"
	"    return vec3(\n"
	"        dot(position - vertices[0], position - vertices[0]),\n"
	"        dot(position - vertices[1], position - vertices[1]),\n"
	"        dot(position - vertices[2], position - vertices[2]));\n"
	"}\n"
	"\n"
	"vec3 computeEdgeDist()\n"
	"{\n"
	"    return vec3(\n"
	"        determinant(mat2(normEdges[0], position - vertices[1])),\n"
	"        determinant(mat2(normEdges[1], position - vertices[2])),\n"
	"        determinant(mat2(normEdges[2], position - vertices[0])));\n"
	"}\n"
	"\n"
	"float irlerp(in vec2 vx, in vec2 v1, in vec2 v2)\n"
	"{\n"
	"    float alpha = acos(clamp(dot(v1, vx), -1., 1.));\n"
	"    float beta = acos(clamp(dot(v1, v2), -1., 1.));\n"
	"    return alpha / beta;\n"
	"}\n"
	"\n"
	"vec4 quadraticInterp(in vec4 colors[6])\n"
	"{\n"
	"    return\n"
	"        colors[0] * linearBasis.x * (2. * linearBasis.x - 1.) +\n"
	"        colors[1] * linearBasis.y * (2. * linearBasis.y - 1.) +\n"
	"        colors[2] * linearBasis.z * (2. * linearBasis.z - 1.) +\n"
	"        colors[3] * 4. * linearBasis.y * linearBasis.z +\n"
	"        colors[4] * 4. * linearBasis.z * linearBasis.x +\n"
	"        colors[5] * 4. * linearBasis.x * linearBasis.y;\n"
	"}\n"
	"\n"
	"float interpFactor(float dist, float radius)\n"
	"{\n"
	"	return clamp(.5 + radius - dist*zoom, 0, 1);\n"
	"}\n"
	"\n"
	"vec4 colorWithBordersAndPoints(in vec4 colorNodes[6])\n"
	"{\n"
	"    vec3 vertexSqrDist = computeVertexSqrDist();\n"
	"    int closestVx = minIndex(vertexSqrDist);\n"
	"\n"
	"    vec3 edgeDist = computeEdgeDist();\n"
	"    int closestEdge = minIndex(edgeDist);\n"
	"\n"
	"    vec4 color = quadraticInterp(colorNodes);\n"
	"\n"
	"    if(showWireframe)\n"
	"    {\n"
	"		color = mix(color, wireframeColor,\n"
	"					interpFactor(edgeDist[closestEdge], halfLineWidth));\n"
	"//        color = mix(color, colorNodes[closestEdge + 3]*.5,\n"
	"//                    interpFactor(edgeDist[closestEdge], halfLineWidth));\n"
	"		color = mix(color, pointColor,\n"
	"					interpFactor(sqrt(vertexSqrDist[closestVx]), pointRadius));\n"
	"//        color = mix(color, colorNodes[closestVx],\n"
	"//                    interpFactor(sqrt(vertexSqrDist[closestVx]), pointRadius));\n"
	"    }\n"
	"\n"
	"    return color;\n"
	"}\n"
	"";

static const char* frag_quadratic_glsl =
	"#version 410 core\n"
	"\n"
	"uniform samplerBuffer nodes;\n"
	"uniform int baseNodeIndex;\n"
	"uniform bool singularTriangles;\n"
	"\n"
	"in vec3 linearBasis;\n"
	"in vec2 position;\n"
	"flat in vec2 vertices[3];\n"
	"flat in vec2 normEdges[3];\n"
	"\n"
	"out vec4 out_color;\n"
	"\n"
	"float irlerp(in vec2 vx, in vec2 v1, in vec2 v2);\n"
	"vec4 quadraticInterp(in vec4 colors[6]);\n"
	"\n"
	"int baseVxIndex = baseNodeIndex + gl_PrimitiveID * (6 + int(singularTriangles));\n"
	"int baseEdgeIndex = baseVxIndex + 3;\n"
	"\n"
	"void main(void)\n"
	"{\n"
	"    vec4 colorNodes[] = vec4[6](\n"
	"        texelFetch(nodes, baseVxIndex + 0),\n"
	"        texelFetch(nodes, baseVxIndex + 1),\n"
	"        texelFetch(nodes, baseVxIndex + 2),\n"
	"        texelFetch(nodes, baseEdgeIndex + 0),\n"
	"        texelFetch(nodes, baseEdgeIndex + 1),\n"
	"        texelFetch(nodes, baseEdgeIndex + 2)\n"
	"    );\n"
	"\n"
	"	if(singularTriangles)\n"
	"	{\n"
	"		colorNodes[0] = mix(colorNodes[0],\n"
	"							texelFetch(nodes, baseVxIndex + 6),\n"
	"							irlerp(normalize(position - vertices[0]),\n"
	"									normEdges[2], -normEdges[1]));\n"
	"	}\n"
	"\n"
	"	out_color = quadraticInterp(colorNodes);\n"
	"}\n"
	"";

static const char* frag_wireframe_glsl =
	"#version 410 core\n"
	"\n"
	"uniform samplerBuffer nodes;\n"
	"uniform float lineWidth;\n"
	"uniform vec4 wireframeColor;\n"
	"\n"
	"in vec3 linearBasis;\n"
	"in vec2 position;\n"
	"flat in vec2 vertices[3];\n"
	"flat in vec2 normEdges[3];\n"
	"\n"
	"out vec4 out_color;\n"
	"\n"
	"vec3 computeEdgeDist();\n"
	"int minIndex(in vec3 dist);\n"
	"float interpFactor(float dist, float radius);\n"
	"\n"
	"void main(void)\n"
	"{\n"
	"	vec3 edgeDist = computeEdgeDist();\n"
	"	int closestEdge = minIndex(edgeDist);\n"
	"	float intensity = interpFactor(edgeDist[closestEdge], lineWidth/2.);\n"
	"	if(intensity < 0.001)\n"
	"		discard;\n"
	"\n"
	"	out_color = vec4(wireframeColor.rgb, wireframeColor.a * intensity);\n"
	"}\n"
	"";

static const char* geom_common_glsl =
	"#version 410 core\n"
	"\n"
	"layout(triangles) in;\n"
	"layout(triangle_strip, max_vertices = 3) out;\n"
	"\n"
	"in vec2 position_obj[];\n"
	"\n"
	"out int gl_PrimitiveID;\n"
	"out vec3 linearBasis;\n"
	"out vec2 position;\n"
	"flat out vec2 vertices[3];\n"
	"flat out vec2 normEdges[3];\n"
	"\n"
	"const vec3 basis[3] = vec3[3](\n"
	"    vec3(1, 0, 0),\n"
	"    vec3(0, 1, 0),\n"
	"    vec3(0, 0, 1)\n"
	");\n"
	"\n"
	"void main()\n"
	"{\n"
	"    for(int i=0; i<3; ++i)\n"
	"    {\n"
	"        gl_Position = gl_in[i].gl_Position;\n"
	"        gl_PrimitiveID = gl_PrimitiveIDIn;\n"
	"        linearBasis = basis[i];\n"
	"        position = position_obj[i];//gl_in[i].gl_Position.xy;\n"
	"        for(int j=0; j<3; ++j)\n"
	"        {\n"
	"            vertices[j] = position_obj[j];//gl_in[j].gl_Position.xy;\n"
	"            normEdges[j] = normalize(vertices[(j+2)%3] - vertices[(j+1)%3]);\n"
	"        }\n"
	"        EmitVertex();\n"
	"    }\n"
	"}\n"
	"";

static const char* vert_common_glsl =
	"#version 410 core\n"
	"\n"
	"uniform mat4 viewMatrix;\n"
	"\n"
	"in vec4 vx_position;\n"
	"\n"
	"out vec2 position_obj;\n"
	"\n"
	"void main(void)\n"
	"{\n"
	"    gl_Position = viewMatrix * vx_position;\n"
	"    position_obj = vx_position.xy;\n"
	"}\n"
	"";

}
}

