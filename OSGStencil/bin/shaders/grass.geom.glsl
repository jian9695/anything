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
out vec4 vsColor;
const float kOscillateDelta = 0.25f;
const float kHalfPi = 1.5707;
const float kPi = 3.1415926;
const float kQuarterPi = 0.7853;
const float kWindCoeff = 0.1f;
uniform float osg_FrameTime;
uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
uniform vec3 cameraPosition;
uniform float ringFlag;

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
	vec3 root = gl_in[0].gl_Position.xyz;
	root.z = 0;
	float h = gs_in[0].bladeHeight;
	float w = gs_in[0].bladeWidth;
	mat3 tiltMatrix = createRotationMatrix(vec3(-1,0,0), gs_in[0].tilt);
  mat3 rotationMatrix = createRotationMatrix(vec3(0,0,1), gs_in[0].rotate);
	mat3 modelMatrix = rotationMatrix * tiltMatrix;
	//modelMatrix[2][0] = root.x;
	//modelMatrix[2][1] = root.y;
	vec3 v0 = modelMatrix * vec3(w * 0.5, 0, 0);
	vec3 v1 = modelMatrix * vec3(-w * 0.5,  0, 0);
	vec3 v2 = modelMatrix * vec3(w * 0.5, h, 0);
	vec3 v3 = modelMatrix * vec3(-w * 0.5,  h, 0);

	uv = vec2(1, 0);
	gl_Position = projection * view * vec4(v0+root, 1);
	EmitVertex();

	uv = vec2(0, 0);
	gl_Position = projection * view * vec4(v1+root, 1);
	EmitVertex();

	uv = vec2(1, 1);
	gl_Position = projection * view * vec4(v2+root, 1);
	EmitVertex();

	uv = vec2(0, 1);
	gl_Position = projection * view * vec4(v3+root, 1);
	EmitVertex();

  EndPrimitive();
}  