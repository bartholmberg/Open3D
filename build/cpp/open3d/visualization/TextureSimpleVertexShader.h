// Automatically generated header file for shader.
// See LICENSE.txt for full license statement.

const char * const TextureSimpleVertexShader = 
"#version 330\n"
"\n"
"in vec3 vertex_position;\n"
"in vec2 vertex_uv;\n"
"uniform mat4 MVP;\n"
"\n"
"out vec2 fragment_uv;\n"
"\n"
"void main()\n"
"{\n"
"    gl_Position = MVP * vec4(vertex_position, 1);\n"
"    fragment_uv = vertex_uv;\n"
"}\n"
;
