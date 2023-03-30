#version 330 core
out vec4 FragColor;

uniform float osg_FrameTime;
uniform mat4 projMat;
uniform mat4 viewMat;

uniform float top;
uniform float left;
uniform float right;
uniform float bottom;
uniform sampler2DArray u_volumeTex;
uniform sampler2D u_texture0;
uniform bool showGrid;
uniform float resolution;
uniform vec3 cameraPosition;
uniform vec2 u_tiltRotate;
uniform vec2 u_rotateTex;
uniform vec3 u_localCenter;

varying vec4 v_color;
varying vec2 v_uv;
varying vec3 v_position;

const float DEG2RAD = 0.0174533;
const float RAD2DEG = 57.2958;

mat4 createLookAt(vec3 eye, vec3 at, vec3 up)
{
  vec3 zaxis = normalize(at - eye); 
  vec3 xaxis = normalize(cross(zaxis, up));
  vec3 yaxis = cross(xaxis, zaxis);

  //negate(zaxis);
  zaxis = -zaxis;
  mat4 viewMatrix = mat4(
    vec4(xaxis.x, yaxis.x, zaxis.x, 0),
    vec4(xaxis.y, yaxis.y, zaxis.y, 0),
    vec4(xaxis.z, yaxis.z, zaxis.z, 0),
    vec4(-dot(xaxis, eye), -dot(yaxis, eye), -dot(zaxis, eye), 1));

  return viewMatrix;
}

mat4 createOrtho(float left, float right, float bottom, float top, float near, float far)
{
    mat4 orthoMat = mat4(
      vec4(2.0 / (right - left), 0, 0, 0),
      vec4(0, 2.0 / (top - bottom), 0, 0),
      vec4(0, 0, -2.0 / (far - near), 0),
      vec4(-(right + left) / (right - left), -(top + bottom) / (top - bottom), -(far + near) / (far - near), 1)
    );
    return orthoMat;
}

//vec3 bbMin = vec3(-0.5, -0.5, 0);
//vec3 bbMax = vec3(0.5, 0.5, 0);

mat4 createOrthoFromBB(mat4 view, vec3 bbMin, vec3 bbMax)
{
  vec3 corners[8];
  corners[0] = bbMin;
  corners[1] = bbMax;
  corners[2] = vec3(bbMin.x, bbMin.y, bbMax.z);
  corners[3] = vec3(bbMin.x, bbMax.y, bbMax.z);
  corners[4] = vec3(bbMin.x, bbMax.y, bbMin.z);
  corners[5] = vec3(bbMax.x, bbMin.y, bbMax.z);
  corners[6] = vec3(bbMax.x, bbMin.y, bbMin.z);
  corners[7] = vec3(bbMax.x, bbMax.y, bbMin.z);

  vec3 bbMin2 = vec3(9999999, 9999999, 9999999);
  vec3 bbMax2 = vec3(-9999999, -9999999, -9999999);
  for(int i=0;i<8;i++)
  {
    vec3 p = (view * vec4(corners[i], 0)).xyz;
    bbMin2 = min(bbMin2, p);
    bbMax2 = max(bbMax2, p);
  }

  float w = max(abs(bbMax2.x), abs(bbMin2.x)) * 1.02;
  float h = max(abs(bbMax2.y), abs(bbMin2.y)) * 1.02;
  mat4 proj = createOrtho(-w, w, -h, h, 1, length(bbMax2) + 1000);
  return proj;
}

mat3 createRotate(vec3 axis, float angle)
{
    axis = normalize(axis);
    float s = sin(angle);
    float c = cos(angle);
    float oc = 1.0 - c;
    
    return mat3(oc * axis.x * axis.x + c,           oc * axis.x * axis.y - axis.z * s,  oc * axis.z * axis.x + axis.y * s,
                oc * axis.x * axis.y + axis.z * s,  oc * axis.y * axis.y + c,           oc * axis.y * axis.z - axis.x * s,
                oc * axis.z * axis.x - axis.y * s,  oc * axis.y * axis.z + axis.x * s,  oc * axis.z * axis.z + c);
}

mat4 createViewFromTiltRotate(vec3 center, float tilt, float rotate, float bbRadius)
{
		mat3 mat = createRotate(vec3(-1, 0, 0), tilt * DEG2RAD);
		mat = createRotate(vec3(0, 0, -1), rotate * DEG2RAD) * mat;

    vec3 oriVec = vec3(0, 1, 0);
    oriVec = normalize(mat * oriVec);
		vec3 oriUp = vec3(0, 0, 1);
	  oriUp = normalize(mat * oriUp);
		vec3 newEye = center + oriVec * (bbRadius + 100);
    mat4 view = createLookAt(newEye, center, oriUp);
    return view;
}

float angleBetween(vec3 v0, vec3 v1)
{   
    if(v0 == v1)
      return 0;
    v0 = normalize(v0);
    v1 = normalize(v1);
		float angle = acos(dot(v0, v1)) * RAD2DEG;
    vec3 n = normalize(cross(v0, v1));
	  float d = dot(vec3(0,0,1), n);
    if(d > 0)
       return angle;
    return 360 - angle;
}

vec2 vectorToTiltRotate(vec3 fromEye)
{
   vec3 right = normalize(cross(fromEye, vec3(0,0,-1)));
   float angleFromUp = angleBetween(fromEye, vec3(0,0,1));
   if(fromEye.z > 0 && (angleFromUp < 10 || angleFromUp > 340))
     return vec2(90, 0);

   float tilt = 90 - angleFromUp;
   mat3 tiltBack = createRotate(right, tilt * DEG2RAD);
   fromEye = tiltBack * fromEye;
   float rotate = 360 - angleBetween(fromEye, vec3(0,-1,0));
   if(tilt < 0)
      tilt = 0;
   return vec2(tilt, rotate);
}

vec3 projectToGround(vec3 fromEye)
{
   fromEye = normalize(fromEye);
   if(fromEye == vec3(0,0,1))
      return fromEye;

   vec3 right = normalize(cross(fromEye, vec3(0,0,1)));
   float angleFromUp = angleBetween(fromEye, vec3(0,0,1));
   float tilt = 90 - angleFromUp;
   mat3 tiltBack = createRotate(right, tilt * DEG2RAD);
   fromEye = tiltBack * fromEye;
   return fromEye;
}

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
    vec4 ofa = texture( samp, (iuv + vec2(0.5,0.5))/512.0 );
    vec4 ofb = texture( samp, (iuv + vec2(1.5,0.5))/512.0 );
    vec4 ofc = texture( samp, (iuv + vec2(0.5,1.5))/512.0 );
    vec4 ofd = texture( samp, (iuv + vec2(1.5,1.5))/512.0 );
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

vec4 textureNoTile2( sampler2D samp, in vec2 uv )
{
    vec2 p = floor( uv );
    vec2 f = fract( uv );
	
    // derivatives (for correct mipmapping)
    vec2 ddx = dFdx( uv );
    vec2 ddy = dFdy( uv );
    
    // voronoi contribution
    vec4 va = vec4( 0.0 );
    float wt = 0.0;
    for( int j=-1; j<=1; j++ )
    for( int i=-1; i<=1; i++ )
    {
        vec2 g = vec2( float(i), float(j) );
        vec4 o = hash4( p + g );
        vec2 r = g - f + o.xy;
        float d = dot(r,r);
        float w = exp(-5.0*d );
        vec4 c = textureGrad( samp, uv + o.zw, ddx, ddy );
        va += w*c;
        wt += w;
    }
	
    // normalization
    return va/wt;
}

void main(void)
{   
  
   vec2 index = floor(vec2((v_position.x - left) / resolution, (top - v_position.y) / resolution));
   vec2 uv = fract(vec2((v_position.x - left) / resolution, (v_position.y - bottom) / resolution));
   vec2 oriUV = fract(vec2((v_position.x - left) / resolution, (v_position.y - bottom) / resolution));
   vec2 center = vec2(left, bottom) + index * resolution + vec2(0.5,0.5) * resolution;
   //vec3 vertexToEye = normalize(vec3(-0.444121778, -0.562221587, 0.697612226) - vec3(0));
   vec3 vertexToEye = normalize(cameraPosition - vec3(0,0,0));
   //vec3 projectedVertexToEye = projectToGround(vertexToEye);
   vec2 tiltRotate = u_tiltRotate;//vectorToTiltRotate(vertexToEye);
   //tiltRotate = vec2(60, 0);
   vec3 bbMin = vec3(-0.5, -0.5, -0.5);
   vec3 bbMax = vec3(0.5, 0.5, 0.5);
   mat4 view = createViewFromTiltRotate(vec3(0,0,0), tiltRotate.x, tiltRotate.y, length(bbMax));
   mat4 proj = createOrthoFromBB(view, bbMin, bbMax);
   float tilt = tiltRotate.x + 0.01;
   float rotate = tiltRotate.y + 0.01;
   float texLayer = 288;
   if(tilt < 80.5)
   {
      float row = floor((tilt - 45) / 10);
      float col = floor(rotate / 10);
      texLayer = row * 36 + col;
   }

   float marchDelta = resolution * 0.5;
   vec3 marchIncrement = vertexToEye * marchDelta;
   vec3 marchPos = v_position.xyz;
   int steps = 0;
   vec4 color = vec4(1,0,0,0);
   /*while(steps < 10)
   {      
      steps++;
      marchPos = marchPos + marchIncrement;
      if(marchPos.x < left || marchPos.y < bottom || marchPos.x > right || marchPos.y > top)
         break;
      vec2 gridIndex = floor(vec2((marchPos.x - left) / resolution, (top - marchPos.y) / resolution));
      if(gridIndex.x == index.x && gridIndex.y == index.y)
        continue;

      vec2 gridCenter = vec2(left, bottom) + gridIndex * resolution + vec2(0.5,0.5) * resolution;
      vec3 posToGridCenter = vec3(marchPos - vec3(gridCenter, v_position.z));
      vec4 projectedCoords = (proj * view * vec4(posToGridCenter, 1));
      vec2 projUV = (projectedCoords.xyz / projectedCoords.w).xy;
      if(projUV.x < -1 || projUV.x > 1 || projUV.y < -1 || projUV.y > 1)
        continue;

      projUV = projUV * 0.5 + vec2(0.5);
      color = texture2DArray(u_volumeTex, vec3(projUV, texLayer));   
   }*/

   /*//if(length(color) < 0.001)
   //{
      vec3 posToGridCenter = vec3((uv - vec2(0.5)) * resolution, v_position.z);
      vec4 projectedCoords = (proj * view * vec4(posToGridCenter, 1));
      vec2 projUV = (projectedCoords.xyz / projectedCoords.w).xy;
      projUV = projUV * 0.5 + vec2(0.5);
      color = texture2DArray(u_volumeTex, vec3(projUV, texLayer));
   //}*/

   //uv = uv - vec2(0.5);
   //uv = uv * 0.5;
   //mat3 ma = createRotate(vec3(0, 0, -1), (360 - u_rotateTex.y) * DEG2RAD);
   //uv = (ma * vec3(uv,0.0)).xy;
   //uv = uv + vec2(0.5);
   //color = texture2DArray(u_volumeTex, vec3(uv, texLayer)) * 2;
   //float texel = 0.001953125;
   /*if(oriUV.x < texel * 5 || oriUV.y < texel * 5 || (1 - oriUV.x) < texel * 5 || (1 - oriUV.y) < texel * 5)
   {
      float o = 2;
      vec4 sumColor = vec4(0);
      for(float y = -o;y <= o;y++)
      {
         float newV = uv.y + y * texel;
         for(float x = -o;x <= o;x++)
         {         
            float newU = uv.x + x * texel;
            sumColor+=(texture(u_texture0, vec2(newU, newV)) * 2);
         }
      }
      color = sumColor / ((o * 2 + 1)*(o * 2 + 1));
   }
   else
   {
      color = texture(u_texture0, uv) * 2;
   }*/
   //color = textureNoTile(u_texture0, uv) * 2;
   //color = textureNoTile2(u_texture0, uv) * 2;
   oriUV = fract(vec2((v_position.x - left) / resolution, (v_position.y - bottom) / resolution));
   //vec2 scale = vec2(0.5, 0.5);
   //oriUV = oriUV * scale;
   //oriUV = oriUV + vec2(0.25);
   color = texture(u_texture0, oriUV);
   color.xy = color.xy + oriUV * 0.6;
   color.a = 1;
   //if(v_position.x > -1.5 && v_position.x < 1.5 && v_position.y > -1.5 && v_position.y < 1.5)
   //   color = vec4(0,0,0,0);
   FragColor = color;
}