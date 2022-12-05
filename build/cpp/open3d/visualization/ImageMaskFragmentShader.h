// Automatically generated header file for shader.
// See LICENSE.txt for full license statement.

const char * const ImageMaskFragmentShader = 
"#version 330\n"
"\n"
"in vec2 UV;\n"
"uniform sampler2D image_texture;\n"
"\n"
"uniform vec3 mask_color;\n"
"uniform float mask_alpha;\n"
"\n"
"out vec4 FragColor;\n"
"\n"
"void main()\n"
"{\n"
"    FragColor = vec4(mask_color, texture(image_texture, UV).r * mask_alpha);\n"
"}\n"
;
