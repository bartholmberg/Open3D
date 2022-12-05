// Automatically generated header file for shader.
// See LICENSE.txt for full license statement.

const char * const ImageVertexShader = 
"#version 330\n"
"\n"
"in vec3 vertex_position;\n"
"in vec2 vertex_UV;\n"
"\n"
"out vec2 UV;\n"
"\n"
"uniform vec3 vertex_scale;\n"
"\n"
"void main()\n"
"{\n"
"    gl_Position = vec4(vertex_position * vertex_scale, 1);\n"
"    UV = vertex_UV;\n"
"}\n"
;
