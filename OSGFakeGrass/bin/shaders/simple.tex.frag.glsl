#version 120
uniform sampler2D u_texture0;
varying vec4 v_color;
void main(void)
{   
    //gl_FragColor = v_color;
    gl_FragColor = texture2D(u_texture0, gl_TexCoord[0].xy);
}