#version 120
varying vec2 texcoord;
varying vec3 position;
uniform float top;
uniform float left;
//uniform float resolution;
void main(void)
{
   gl_TexCoord[0] = gl_MultiTexCoord0;
   texcoord = gl_MultiTexCoord0.xy;
   position = gl_Vertex.xyz;
   gl_Position = ftransform();
}