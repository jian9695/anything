#version 330 core

layout (location = 0) in vec3 osg_Vertex;
layout (location = 1) in vec2 osg_MultiTexCoord0;

//OSG input layout
//layout (location = 0) in vec4 osg_Vertex;
//layout (location = 1) in vec3 osg_Normal;
//layout (location = 2) in vec4 osg_Color;
//layout (location = 8) in vec4 osg_MultiTexCoord0;
//layout (location = 4) in vec4 osg_MultiTexCoord1;

uniform float osg_FrameTime;
uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

out vec2 uv;

void main() {
    uv = osg_MultiTexCoord0.xy;
    uv = osg_Vertex.xy;
    gl_Position = projection * view * model * vec4(osg_Vertex, 1);        
}

0.001
0.0015875