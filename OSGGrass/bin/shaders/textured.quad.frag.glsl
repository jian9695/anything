#version 330 core
out vec4 FragColor;

uniform float osg_FrameTime;
uniform mat4 projMat;
uniform mat4 viewMat;
uniform mat4 u_renderViewProjInverse;

uniform float top;
uniform float left;
uniform float right;
uniform float bottom;
uniform sampler2DArray u_volumeTex;
uniform sampler2D u_texture0;
uniform sampler2D u_texture1;
uniform bool u_showGrid;
uniform float u_resolution;
uniform vec3 cameraPosition;
uniform vec2 u_tiltRotate;
uniform vec2 u_rotateTex;
uniform vec3 u_localCenter;
uniform mat4 u_localTransform;
uniform mat4 u_localTransformInverse;
uniform vec2 u_texSize;
uniform vec3 u_renderEye;
uniform vec3 u_renderLook;
uniform float u_near;
uniform float u_far;
uniform vec4 u_maskExtent;

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

void main(void)
{   
   vec3 pos = (u_localTransform * vec4(v_position,0)).xyz;
   vec2 bbMin = vec2(-1000, -1000);
   vec2 uv = (pos.xy - bbMin) / u_resolution;
   vec2 index = floor(uv);
   uv = fract(uv);
   vec3 center = vec3(bbMin + index * u_resolution + vec2(0.5,0.5) * u_resolution, 0);
   center = (u_localTransformInverse * vec4(center, 0)).xyz;
   vec2 clipSpace = uv * 2.0 - vec2(1.0);
   clipSpace.y = 1 - clipSpace.y;


   float depth = texture(u_texture1, uv).x;
   vec4 color = texture(u_texture0, uv);
   if(u_showGrid)
      color.rg += uv;
   vec3 adjuestedPos = (u_renderViewProjInverse * vec4(clipSpace.x, clipSpace.y, depth, 1.0)).xyz;
   //color = vec4(adjuestedPos.z/0.1, 0, 0, 1);
   //color.rgb = normalize(center);
   adjuestedPos += center;
   vec4 projectedPos = projMat * viewMat * vec4(adjuestedPos, 1);
   float adjuestedDepth = projectedPos.z / projectedPos.w;
   //adjuestedDepth = (((u_far-u_near) * adjuestedDepth) + u_near + u_far) / 2.0;
   //gl_FragDepth = ((gl_DepthRange.diff * adjuestedDepth) + gl_DepthRange.near + gl_DepthRange.far) / 2.0;

   //adjuestedDepth = (1.0 - 0.0) * 0.5 * adjuestedDepth + (1.0 + 0.0) * 0.5;
   //gl_FragDepth = adjuestedDepth;

   color.a = 1;
   if(!isnan(u_maskExtent.x) && v_position.x > u_maskExtent.x && v_position.x < u_maskExtent.z && v_position.y > u_maskExtent.y && v_position.y < u_maskExtent.w)
      color = vec4(0,0,0,0);
   FragColor = color;
}