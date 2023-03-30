#version 330 core

layout (location = 0) in vec3 osg_Vertex;
layout (location = 1) in vec4 osg_MultiTexCoord0;

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

out VS_OUT {
    float rotate;
    float tilt;
		float bladeWidth;
		float bladeHeight;
} vs_out;

out vec2 uv;

void main() {
    uv = osg_MultiTexCoord0.xy;
		vec4 root = vec4(osg_Vertex.xyz, 1.0);
    vs_out.rotate = osg_MultiTexCoord0.x;
    vs_out.tilt = osg_MultiTexCoord0.y;
    vs_out.bladeWidth = osg_MultiTexCoord0.z;
    vs_out.bladeHeight = osg_MultiTexCoord0.w;
    
    gl_Position = root;
    //gl_Position = root;
    //gl_Position = ftransform();         
}