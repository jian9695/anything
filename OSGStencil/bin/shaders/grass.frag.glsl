#version 330 core
out vec4 FragColor;
in vec4 vsColor;
in vec2 uv; // the input variable from the vertex shader (same name and same type) 
uniform sampler2D u_grassTex;
uniform sampler2D u_grassAlpha;
uniform float osg_FrameTime;

void main() {
    vec4 color = texture2D(u_grassTex, uv);
    vec4 alpha = texture2D(u_grassAlpha, uv);
    //FragColor = vec4(uv.x, uv.y, 0, alpha.x);
    //FragColor = vec4(vsColor.rgb, 1);
    FragColor = vec4(color.rgb * vec3(0.15,1,0.15), alpha.x > 0.1 ? 1 : 0);
    //FragColor = vec4(vec3(1,0,0), alpha.x > 0.1 ? 1 : 0);
    //FragColor = vec4(uv.x, uv.y, 0, alpha.x);
    //FragColor = vsColor;
}