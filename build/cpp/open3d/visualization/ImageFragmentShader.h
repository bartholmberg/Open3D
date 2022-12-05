// Automatically generated header file for shader.
// See LICENSE.txt for full license statement.

const char * const ImageFragmentShader = 
"#version 330\n"
"\n"
"in vec2 UV;\n"
"uniform sampler2D image_texture;\n"
"\n"
"out vec4 FragColor;\n"
"\n"
"void main()\n"
"{\n"
"    FragColor = texture(image_texture, UV);\n"
"}\n"
;
