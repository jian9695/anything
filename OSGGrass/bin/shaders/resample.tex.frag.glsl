#version 330 core
out vec4 FragColor;
uniform sampler2D u_texture0;
uniform float u_reps;

varying vec2 v_uv;

void main(void)
{  
   vec4 color = texture(u_texture0, v_uv * u_reps);
   FragColor = color;
}