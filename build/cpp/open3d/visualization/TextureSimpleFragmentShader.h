// Automatically generated header file for shader.
// See LICENSE.txt for full license statement.

const char * const TextureSimpleFragmentShader = 
"#version 330\n"
"\n"
"in vec2 fragment_uv;\n"
"out vec4 FragColor;\n"
"\n"
"uniform sampler2D diffuse_texture;\n"
"\n"
"void main()\n"
"{\n"
"    FragColor = texture(diffuse_texture, fragment_uv);\n"
"}\n"
;
