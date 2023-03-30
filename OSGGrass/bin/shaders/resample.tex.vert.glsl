#version 330 core

layout (location = 0) in vec3 osg_Vertex;

varying vec2 v_uv;

void main(void)
{        
   v_uv = vec4(gl_Vertex.x*0.5+0.5,gl_Vertex.y*0.5+0.5);
   gl_Position = vec4(gl_Vertex.x,gl_Vertex.y,0,1.0);
}