#version 330 core
layout (points) in;
layout (triangle_strip, max_vertices = 54) out;

struct VertexOut {
	vec4 Position;
	vec2 TexCoord;
	vec4 Normal;
	vec3 VertexToLight;
	vec3 VertexToCamera;
	vec3 LevelOfDetail;
	float Random;
};

in VS_OUT {
    float rotate;
    float tilt;
		float bladeWidth;
		float bladeHeight;
} gs_in[];

out VertexOut VSOut;
out vec2 uv;
out vec3 normal;
out vec3 position;
out vec4 vsColor;
out vec4 screenPos;
const float kOscillateDelta = 0.25f;
const float kHalfPi = 1.5707;
const float kPi = 3.1415926;
const float kQuarterPi = 0.7853;
const float kWindCoeff = 0.1f;
const float kDegToRad = 0.0174533f;
const int kNumVertices = 6;

uniform float osg_FrameTime;
uniform mat4 projection;
uniform mat4 view;
uniform vec3 cameraPosition;
uniform float lod;
uniform float minBladeWidth;
uniform float maxBladeWidth;
uniform float minBladeHeight;
uniform float maxBladeHeight;
uniform float minTilt;
uniform float maxTilt;
uniform float minRotate;
uniform float maxRotate;
uniform float top;
uniform float left;
uniform float right;
uniform float bottom;
//uniform float gridSize;
uniform bool u_animate;
uniform vec3 u_translate;
uniform vec3 u_groupTranslate;

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

mat3 createRotationMatrix(vec3 axis, float angle)
{
    axis = normalize(axis);
    float s = sin(angle);
    float c = cos(angle);
    float oc = 1.0 - c;
    
    return mat3(oc * axis.x * axis.x + c,           oc * axis.x * axis.y - axis.z * s,  oc * axis.z * axis.x + axis.y * s,
                oc * axis.x * axis.y + axis.z * s,  oc * axis.y * axis.y + c,           oc * axis.y * axis.z - axis.x * s,
                oc * axis.z * axis.x - axis.y * s,  oc * axis.y * axis.z + axis.x * s,  oc * axis.z * axis.z + c);
}

float genRan(vec2 st, float seed) {
    return fract(sin(dot(st.xy, vec2(12.9898,78.233))) * seed);
}

void main() {

  vec3 toEye = vec3(1, 1, 1);
  vec3 focus = vec3(0, 0, 0);
  vec3 eye = focus + toEye * 2.0;
  mat4 u_view = view;//createLookAt(eye, focus, vec3(0, 1, 0));
  mat4 u_proj = projection;//createOrtho(-0.5,0.5,-0.5,0.5,0.1,500);

	vec3 xyz = gl_in[0].gl_Position.xyz;
	vec3 root = gl_in[0].gl_Position.xyz;
	root.x = left + genRan(xyz.xy, 43658.5453163);
	root.y = bottom + genRan(xyz.xy, 63918.5657103);
	root.z = 0;
	//vec4 projCoords = projection * view * vec4(root, 1);
	//projCoords.xyz = projCoords.xyz / projCoords.w;
	//if(projCoords.x < -1 || projCoords.x > 1 || projCoords.y < -1 || projCoords.y > 1)
	//   return;
	float h = minBladeHeight + (maxBladeHeight - minBladeHeight) * genRan(xyz.xy, 73289.5473661);
	float w = minBladeWidth + (maxBladeWidth - minBladeWidth) * genRan(xyz.xy, 14567.5443867);
	float tilt = (minTilt + (maxTilt - minTilt) * genRan(xyz.xy, 73289.5473661)) * kDegToRad;
	float rotate = (minRotate + (maxRotate - minRotate) * genRan(xyz.xy, 73289.5473661)) * kDegToRad;
	mat3 tiltMatrix = createRotationMatrix(vec3(-1,0,0), tilt);
  mat3 rotationMatrix = createRotationMatrix(vec3(0,0,1), rotate);
	mat3 modelMatrix = rotationMatrix * tiltMatrix;
	//modelMatrix[2][0] = root.x;
	//modelMatrix[2][1] = root.y;
	vec3 v0 = modelMatrix * vec3(w * 0.5, 0, 0);
	vec3 v1 = modelMatrix * vec3(-w * 0.5,  0, 0);
	vec3 v2 = modelMatrix * vec3(w * 0.5, h, 0);
	vec3 v3 = modelMatrix * vec3(-w * 0.5,  h, 0);
	normal = normalize(modelMatrix * vec3(0, 0, 1));

	vec2 windVec = vec2(1, 1);
	float time = osg_FrameTime;
	float phase = sin(time * kPi * 0.25 + genRan(xyz.xy, 43658.5453163) * kPi * 0.25);
	float perimeter = h * 2;
	//float r = perimeter / (2 * 3.14159);
	float ry = h * (0.25 + genRan(xyz.xz, 143651.585343) * 0.2);
	float rx = sqrt(pow(perimeter / (2 * 3.14159), 2) * 2 - ry * ry);
	//ry = rx = r;
	float cx = rx;
	float cy = 0;

	//int win = 1;
  //for(float row = -win;row <= win; row++)
	//{
  //for(float col = -win;col <= win; col++)
	//{
	//vec3 translate = vec3(col*gridSize, row*gridSize, 0);
	root = gl_in[0].gl_Position.xyz + u_translate + u_groupTranslate;
	root.z = 0;

	uv.y = 0;
	float deltaV = 1.0 / (kNumVertices / 2 - 1.0);
	for(int v=0; v<kNumVertices/2;v++)
	{
		for(int u=0;u<2;u++)
		{
			uv.x = float(u);
			float hSign = (u == 0) ? 1 : -1;
			float windCoEff = uv.y;

			float theta = (180.0f - 90.0f * windCoEff) * 3.1415926 / 180.0;
			vec2 pa = vec2(0, windCoEff * h);
			vec2 pb = vec2(cos(theta) * rx + cx, sin(theta) * ry + cy);
			vec2 v0 = normalize(pa - pb);
			vec2 v1 = vec2(-1, 0);
			float angleBetween = acos(dot(v0, v1) / (length(v0) * length(v1)));
		
			vec3 windir3d = normalize(vec3(windVec.x, windVec.y, 0));
			vec3 axis = cross(windir3d, vec3(0, 0, -1));
			mat3 rotate = createRotationMatrix(axis, angleBetween);
			windir3d = normalize(windir3d * rotate);

			position = modelMatrix * vec3(w * 0.5 * hSign, uv.y * h, 0);

			if(u_animate)
			   position = position + windir3d * length(pb-pa) * (phase * 0.5 + 0.5);

			position = position + root;
			gl_Position = u_proj * u_view * vec4(position, 1);
			screenPos.xyz = position;
			EmitVertex();
		}
	  uv.y+=deltaV;
	}

 EndPrimitive();
	//}	
	//}
}  