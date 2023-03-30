#version 330 core
out vec4 FragColor;
in vec4 vsColor;
in vec2 uv; // the input variable from the vertex shader (same name and same type) 
uniform sampler2D u_grassTex;
uniform float osg_FrameTime;
#define USEHASH   
vec4 hash4( vec2 p ) { return fract(sin(vec4( 1.0+dot(p,vec2(37.0,17.0)), 
                                              2.0+dot(p,vec2(11.0,47.0)),
                                              3.0+dot(p,vec2(41.0,29.0)),
                                              4.0+dot(p,vec2(23.0,31.0))))*103.0); }

vec4 textureNoTile( sampler2D samp, in vec2 uv )
{
    vec2 iuv = floor( uv );
    vec2 fuv = fract( uv );

#ifdef USEHASH    
    // generate per-tile transform (needs GL_NEAREST_MIPMAP_LINEARto work right)
    vec4 ofa = texture( samp, (iuv + vec2(0.5,0.5))/256.0 );
    vec4 ofb = texture( samp, (iuv + vec2(1.5,0.5))/256.0 );
    vec4 ofc = texture( samp, (iuv + vec2(0.5,1.5))/256.0 );
    vec4 ofd = texture( samp, (iuv + vec2(1.5,1.5))/256.0 );
#else
    // generate per-tile transform
    vec4 ofa = hash4( iuv + vec2(0.0,0.0) );
    vec4 ofb = hash4( iuv + vec2(1.0,0.0) );
    vec4 ofc = hash4( iuv + vec2(0.0,1.0) );
    vec4 ofd = hash4( iuv + vec2(1.0,1.0) );
#endif
    
    vec2 ddx = dFdx( uv );
    vec2 ddy = dFdy( uv );

    // transform per-tile uvs
    ofa.zw = sign(ofa.zw-0.5);
    ofb.zw = sign(ofb.zw-0.5);
    ofc.zw = sign(ofc.zw-0.5);
    ofd.zw = sign(ofd.zw-0.5);
    
    // uv's, and derivarives (for correct mipmapping)
    vec2 uva = uv*ofa.zw + ofa.xy; vec2 ddxa = ddx*ofa.zw; vec2 ddya = ddy*ofa.zw;
    vec2 uvb = uv*ofb.zw + ofb.xy; vec2 ddxb = ddx*ofb.zw; vec2 ddyb = ddy*ofb.zw;
    vec2 uvc = uv*ofc.zw + ofc.xy; vec2 ddxc = ddx*ofc.zw; vec2 ddyc = ddy*ofc.zw;
    vec2 uvd = uv*ofd.zw + ofd.xy; vec2 ddxd = ddx*ofd.zw; vec2 ddyd = ddy*ofd.zw;
        
    // fetch and blend
    vec2 b = smoothstep(0.25,0.75,fuv);
    
    return mix( mix( textureGrad( samp, uva, ddxa, ddya ), 
                     textureGrad( samp, uvb, ddxb, ddyb ), b.x ), 
                mix( textureGrad( samp, uvc, ddxc, ddyc ),
                     textureGrad( samp, uvd, ddxd, ddyd ), b.x), b.y );
}

void main() {
    vec2 uv2 = (uv + vec2(50, 50)) / vec2(10, 10);
    //uv2 = fract(uv2);
    //uv2 = (uv2 * 0.8) + vec2(0.1);
    vec4 color = textureNoTile(u_grassTex, uv2);
    FragColor = color;
}