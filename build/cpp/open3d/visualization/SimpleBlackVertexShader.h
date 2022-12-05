// Automatically generated header file for shader.
// See LICENSE.txt for full license statement.

const char * const SimpleBlackVertexShader = 
"#version 330\n"
"\n"
"in vec3 vertex_position;\n"
"uniform mat4 MVP;\n"
"\n"
"void main()\n"
"{\n"
"    gl_Position = MVP * vec4(vertex_position, 1);\n"
"}\n"
;
