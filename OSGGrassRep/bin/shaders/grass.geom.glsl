#version 330 core
layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

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
const float kOscillateDelta = 0.25f;
const float kHalfPi = 1.5707;
const float kPi = 3.1415926;
const float kQuarterPi = 0.7853;
const float kWindCoeff = 0.1f;
const float kDegToRad = 0.0174533f;
uniform float osg_FrameTime;
uniform mat4 projection;
uniform mat4 view;
uniform vec3 translate;
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
	vec3 xyz = gl_in[0].gl_Position.xyz;
	vec3 root = gl_in[0].gl_Position.xyz + translate;
	//root.x = left + genRan(xyz.xy, 43658.5453163);
	//root.y = bottom + genRan(xyz.xy, 63918.5657103);
	root.z = 0;
	vec4 projCoords = projection * view * vec4(root, 1);
	projCoords.xyz = projCoords.xyz / projCoords.w;
	if(projCoords.x < -1 || projCoords.x > 1 || projCoords.y < -1 || projCoords.y > 1)
	   return;
	float h = minBladeHeight + (maxBladeHeight - minBladeHeight) * genRan(xyz.zy, 73289.5473661);
	float w = minBladeWidth + (maxBladeWidth - minBladeWidth) * genRan(xyz.zy, 14567.5443867);
	float tilt = (minTilt + (maxTilt - minTilt) * genRan(xyz.zy, 73289.5473661)) * kDegToRad;
	float rotate = (minRotate + (maxRotate - minRotate) * genRan(xyz.zy, 73289.5473661)) * kDegToRad;
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

	uv = vec2(1, 0);
	position = v0+root;
	gl_Position = projection * view * vec4(v0+root, 1);
	EmitVertex();

	uv = vec2(0, 0);
	position = v1+root;
	gl_Position = projection * view * vec4(v1+root, 1);
	EmitVertex();

	uv = vec2(1, 1);
	position = v2+root;
	gl_Position = projection * view * vec4(v2+root, 1);
	EmitVertex();

	uv = vec2(0, 1);
	position = v3+root;
	gl_Position = projection * view * vec4(v3+root, 1);
	EmitVertex();

  EndPrimitive();


		/*float theta = (180.0f - 90.0f * windCoEff) * 3.1415926 / 180.0;
		vec2 pa = vec2(0, windCoEff * sizeY);
    vec2 pb = vec2(cos(theta) * rx + cx, sin(theta) * ry + cy);
		vec2 v0 = normalize(pa - pb);
		vec2 v1 = vec2(-1, 0);
		float angleBetween = acos(dot(v0, v1) / (length(v0) * length(v1)));
		
		vec3 windir3d = normalize(vec3(windVec.x, windVec.y, 0));
		vec3 axis = cross(windir3d, vec3(0, 0, -1));
		mat3 rotate = createRotationMatrix(axis, angleBetween);
		windir3d = normalize(windir3d * rotate);
		//vec3 animatedPos = v[i].Position.xyz + vec3(WindVector.x, WindVector.y, 0) * windCoEff *  (sizeY * 0.3) * phase;
		//v[i].Position.xyz = v[i].Position.xyz + windir3d * length(pb-pa) * (phase * 0.5 + 0.5);*/
}  