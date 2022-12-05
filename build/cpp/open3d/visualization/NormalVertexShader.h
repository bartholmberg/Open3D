// Automatically generated header file for shader.
// See LICENSE.txt for full license statement.

const char * const NormalVertexShader = 
"#version 330\n"
"\n"
"in vec3 vertex_position;\n"
"in vec3 vertex_normal;\n"
"\n"
"out vec3 vertex_normal_camera;\n"
"\n"
"uniform mat4 MVP;\n"
"uniform mat4 V;\n"
"uniform mat4 M;\n"
"\n"
"void main()\n"
"{\n"
"    gl_Position = MVP * vec4(vertex_position, 1);\n"
"    vertex_normal_camera = (V * M * vec4(vertex_normal, 0)).xyz;\n"
"}\n"
;
