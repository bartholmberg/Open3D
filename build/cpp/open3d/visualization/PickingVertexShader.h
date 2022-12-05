// Automatically generated header file for shader.
// See LICENSE.txt for full license statement.

const char * const PickingVertexShader = 
"#version 330\n"
"\n"
"in vec3 vertex_position;\n"
"in float vertex_index;\n"
"uniform mat4 MVP;\n"
"\n"
"out vec4 fragment_color;\n"
"\n"
"void main()\n"
"{\n"
"    float r, g, b, a;\n"
"    gl_Position = MVP * vec4(vertex_position, 1);\n"
"    r = floor(vertex_index / 16777216.0) / 255.0;\n"
"    g = mod(floor(vertex_index / 65536.0), 256.0) / 255.0;\n"
"    b = mod(floor(vertex_index / 256.0), 256.0) / 255.0;\n"
"    a = mod(vertex_index, 256.0) / 255.0;\n"
"    fragment_color = vec4(r, g, b, a);\n"
"}\n"
;
