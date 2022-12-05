// Automatically generated header file for shader.
// See LICENSE.txt for full license statement.

const char * const NormalFragmentShader = 
"#version 330\n"
"\n"
"in vec3 vertex_normal_camera;\n"
"out vec4 FragColor;\n"
"\n"
"void main()\n"
"{\n"
"    FragColor = vec4(vertex_normal_camera * 0.5 + 0.5, 1);\n"
"}\n"
;
