// Automatically generated header file for shader.
// See LICENSE.txt for full license statement.

const char * const SimpleVertexShader = 
"#version 330\n"
"\n"
"in vec3 vertex_position;\n"
"in vec3 vertex_color;\n"
"uniform mat4 MVP;\n"
"\n"
"out vec3 fragment_color;\n"
"\n"
"void main()\n"
"{\n"
"    gl_Position = MVP * vec4(vertex_position, 1);\n"
"    fragment_color = vertex_color;\n"
"}\n"
;
