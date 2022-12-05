// Automatically generated header file for shader.
// See LICENSE.txt for full license statement.

const char * const Simple2DVertexShader = 
"#version 330\n"
"\n"
"in vec3 vertex_position;\n"
"in vec3 vertex_color;\n"
"\n"
"out vec3 fragment_color;\n"
"\n"
"void main()\n"
"{\n"
"    gl_Position = vec4(vertex_position, 1);\n"
"    fragment_color = vertex_color;\n"
"}\n"
;
