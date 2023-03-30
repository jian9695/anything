#version 120
varying vec2 texcoord;
varying vec3 position;
uniform float top;
uniform float left;
uniform float right;
uniform float bottom;
uniform sampler2D u_grassSample;
uniform sampler2D u_grassCircle;
uniform sampler2D u_terrainTex;
uniform mat4 texProjection;
uniform mat4 texView;
uniform bool showGrid;
uniform float resolution;
uniform vec3 cameraPosition;
uniform vec3 circleCenter;
uniform float circleRadius;
uniform mat4 projection;
uniform mat4 view;
void main(void)
{
    float resol = resolution;
    vec2 uv = fract(vec2((position.x - left) / resolution, (position.y - bottom) / resolution));
    vec2 oriUV = uv;
    float distToCircle = length(position.xy - circleCenter.xy);
    vec4 color = vec4(0.5,0.5,0.5,1.0);
    if(distToCircle > circleRadius)
    {  
      uv = uv - vec2(0.5);
      uv = uv * resolution;
      uv = (texProjection * texView * vec4(uv.x,uv.y, 0, 1)).xy;
      uv = uv * 0.5 + vec2(0.5);
      float texStep = 1.0/1024;
      color = texture2D(u_grassSample, uv);
      if(cameraPosition.z < 300)
      {    
          float resol = circleRadius * 0.25;
          vec2 uvShift = fract(vec2((position.x - left) / resol, (position.y - bottom) / resol));
          uvShift = (uvShift - vec2(0.5)) * resol;
          vec4 projectedCoords = (projection * view * vec4(circleCenter.x+uvShift.x,circleCenter.y+uvShift.y, 0, 1));
          uv = (projectedCoords.xyz / projectedCoords.w).xy;
          uv = uv * 0.5 + vec2(0.5);
          vec4 color2 = texture2D(u_grassCircle, uv);
          float frac = (cameraPosition.z - 100) / 100;
          if(frac > 1)
             frac = 1;
          color = mix(color, color2, 1.0 - frac);
      }
    }
    else
    {
      if(cameraPosition.z < 300)
      {
          vec4 projectedCoords = (projection * view * vec4(position.x,position.y, 0, 1));
          uv = (projectedCoords.xyz / projectedCoords.w).xy;
          uv = uv * 0.5 + vec2(0.5);
          color = texture2D(u_grassCircle, uv);
      }
      else
      {
         uv = uv - vec2(0.5);
         uv = uv * resolution;
         uv = (texProjection * texView * vec4(uv.x,uv.y, 0, 1)).xy;
         uv = uv * 0.5 + vec2(0.5);
         float texStep = 1.0/1024;
         color = texture2D(u_grassSample, uv);    
      }
    }

    if(color.a < 0.1)
    {
      color = texture2D(u_terrainTex, oriUV);
      color.a = 1;
    }

    vec2 gridColor = vec2(oriUV.x,oriUV.y);
    if(showGrid)
    { 
      color.xy = color.xy + gridColor * 0.2;
    }
    gl_FragColor = color;
    //gl_FragColor = vec4(0.5,0.5,0.5,1.0);
}