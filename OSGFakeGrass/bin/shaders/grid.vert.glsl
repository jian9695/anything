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
uniform mat4 projMat;
uniform mat4 viewMat;

uniform float top;
uniform float left;
uniform float right;
uniform float bottom;
uniform sampler2DArray u_volumeTex;
uniform bool showGrid;
uniform float resolution;
uniform vec3 cameraPosition;
uniform vec2 u_tiltRotate;
uniform vec2 u_rotateTex;
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
    v0 = normalize(v0);
    v1 = normalize(v1);
    vec3 up = vec3(0,0,1);
		float angle = acos(dot(v0, v1)) * RAD2DEG;
    if(cross(v0, v1).z > 0)
       return angle;
    return 360 - angle;
}

vec2 vectorToTiltRotate(vec3 fromEye)
{
   vec3 right = normalize(cross(fromEye, vec3(0,0,1)));
   float angleFromUp = angleBetween(fromEye, vec3(0,0,1));
   //if(fromEye.z > 0 && angleFromUp < 5)
   //  return vec2(90, 0);

   float tilt = 90 - angleFromUp;
   mat3 tiltBack = createRotate(right, tilt * DEG2RAD);
   fromEye = tiltBack * fromEye;
   float rotate = angleBetween(fromEye, vec3(0,1,0));
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

void main(void)
{
   vec2 index = floor(vec2((osg_Vertex.x - left) / resolution, (top - osg_Vertex.y) / resolution));
   vec2 uv = fract(vec2((osg_Vertex.x - left) / resolution, (osg_Vertex.y - bottom) / resolution));
   vec2 center = vec2(left, bottom) + index * resolution + vec2(0.5,0.5) * resolution;
   v_position = osg_Vertex.xyz;

   //vec3 rotatedPos = osg_Vertex.xyz;
   //mat3 ma = createRotate(vec3(0, 0, -1), u_rotateTex.y * DEG2RAD);
   //rotatedPos = ma * rotatedPos;

   gl_Position = projMat * viewMat * vec4( osg_Vertex.xyz, 1);
   /*
   vec2 index = floor(vec2((osg_Vertex.x - left) / resolution, (top - osg_Vertex.y) / resolution));
   vec2 uv = fract(vec2((osg_Vertex.x - left) / resolution, (osg_Vertex.y - bottom) / resolution));
   uv = fract(vec2((osg_Vertex.x - (-1000)) / 1.0, (osg_Vertex.y - (-1000)) / 1.0));
   v_uv = uv;
   vec2 center = vec2(left, bottom) + index * resolution + vec2(0.5,0.5) * resolution;
   vec3 vertexToEye = normalize(cameraPosition - osg_Vertex.xyz);
   //vec3 projectedVertexToEye = projectToGround(vertexToEye);
   vec2 tiltRotate = vectorToTiltRotate(vertexToEye);
   vec3 bbMin = vec3(-0.5, -0.5, -0.5);
   vec3 bbMax = vec3(0.5, 0.5, 0.5);
   mat4 view = createViewFromTiltRotate(vec3(0,0,0), tiltRotate.x, tiltRotate.y, length(bbMax));
   mat4 proj = createOrthoFromBB(view, bbMin, bbMax);
   float tilt = tiltRotate.x + 0.01;
   float rotate = tiltRotate.y + 0.01;
   int texLayer = 288;
   if(tilt < 80.5)
   {
      highp int row = int(floor(tilt / 10));
      highp int col = int(floor(rotate / 10));
      texLayer = row * 36 + col;
   }
   
   vec2 cellOffsets[9];
   cellOffsets[0] = vec2(-1,-1);
   cellOffsets[1] = vec2(0, -1);
   cellOffsets[2] = vec2(1, -1);
   cellOffsets[3] = vec2(-1,0);
   cellOffsets[4] = vec2(1, 0);
   cellOffsets[5] = vec2(-1,1);
   cellOffsets[6] = vec2(0, 1);
   cellOffsets[7] = vec2(1, 1);
   cellOffsets[8] = vec2(0, 0);
   for(int i=0;i<9;i++)
   {  
      vec2 gridIndex = index + cellOffsets[i];
      if(gridIndex.x < 0 || gridIndex.y < 0)
         continue;
      vec2 gridCenter = center + cellOffsets[i] * resolution;
      if(gridCenter.x > right || gridCenter.y > top)
         continue;

      vec2 posToGridCenter = osg_Vertex.xy - gridCenter;
      vec4 projectedCoords = (proj * view * vec4(posToGridCenter.x,posToGridCenter.y, 0, 1));
      uv = (projectedCoords.xyz / projectedCoords.w).xy;
      if(uv.x < -1 || uv.x > 1 || uv.y < -1 || uv.y > 1)
        continue;
      uv = uv * 0.5 + vec2(0.5);
      v_color = texture2DArray(u_volumeTex, vec3(uv, texLayer));
      if(v_color.a > 0.5)
        break;
   }

   
   
   vec2 cellOffsets[9];
   cellOffsets[0] = vec2(-1,-1);
   cellOffsets[1] = vec2(0, -1);
   cellOffsets[2] = vec2(1, -1);
   cellOffsets[3] = vec2(-1,0);
   cellOffsets[4] = vec2(1, 0);
   cellOffsets[5] = vec2(-1,1);
   cellOffsets[6] = vec2(0, 1);
   cellOffsets[7] = vec2(1, 1);
   cellOffsets[8] = vec2(0, 0);
   vec4 color = vec4(1,0,0,1);
   for(int i=0;i<9;i++)
   {  
      vec2 gridIndex = index + cellOffsets[i];
      if(gridIndex.x < 0 || gridIndex.y < 0)
         continue;
      vec2 gridCenter = center + cellOffsets[i] * resolution;
      if(gridCenter.x > right || gridCenter.y > top)
         continue;

      vec3 posToGridCenter = vec3(v_position.xy - gridCenter, 0.5);
      posToGridCenter = vec3(v_position.xy - gridCenter, 0.5);
      vec4 projectedCoords = (proj * view * vec4(posToGridCenter.x,posToGridCenter.y, 0.5, 1));
      vec2 projUV = (projectedCoords.xyz / projectedCoords.w).xy;
      if(projUV.x < -1 || projUV.x > 1 || projUV.y < -1 || projUV.y > 1)
        continue;
      color = vec4(0,1,0,1);
      projUV = projUV * 0.5 + vec2(0.5);
      color = texture2DArray(u_volumeTex, vec3(projUV, texLayer));
      if(color.a > 0.5)
        break;
   }
   v_color = vec4(v_uv.x,v_uv.y,0,1);
   //v_uv = uv;*/
}