#version 330 core
out vec4 FragColor;
in vec4 vsColor;
in vec2 uv; // the input variable from the vertex shader (same name and same type) 
in vec3 position;
in vec3 normal;
in vec4 screenPos;
uniform sampler2D u_grassTex;
uniform sampler2D u_grassAlpha;
uniform float osg_FrameTime;
uniform vec3 cameraPosition;
uniform vec3 circleCenter;
uniform float circleRadius;
uniform bool isSample;
uniform bool u_light;

float genRan(vec2 st, float seed) {
    return fract(sin(dot(st.xy, vec2(12.9898,78.233))) * seed);
}

vec3 HUEtoRGB(float H)
{
	float R = abs(H * 6 - 3) - 1;
	float G = 2 - abs(H * 6 - 2);
	float B = 2 - abs(H * 6 - 4);
	return clamp(vec3(R, G, B), 0.0, 1.0);
}

vec3 HSVtoRGB(vec3 HSV)
{
	vec3 RGB = HUEtoRGB(HSV.x);
	return ((RGB - 1) * HSV.y + 1) * HSV.z;
}

void main() {

    vec3 pos = position;
		//if(isSample)
		//{
		//  pos = circleCenter;// + normalize(pos - circleCenter) * circleRadius;
		//}
    vec3 lightDir = normalize(vec3(1.0, 1.0, 0.5));
    vec3 lightPos = pos + lightDir * 10000;
		float dn1 = dot(lightPos, normal);
		float dn2 = dot(lightPos, -normal);
    vec3 n = normalize(normal);
		if(dn2 < dn1)
		   n = -n;
    vec3 l = normalize(lightDir);
    vec3 vertexToCamera = normalize(cameraPosition - pos);
		vec3 vertexToLight = normalize(pos - lightPos);
    float intensity = max(dot(n,l), 0.0);

		// Phong
		vec3 r = normalize(reflect(vertexToLight, n));
		float shininess = 100;

		float ambientLight = 1.0;
		float diffuseLight = clamp(dot(vertexToLight, n), 0.0, 1.0);
		float specularLight = clamp(dot(-vertexToCamera, r), 0.0, 1.0);
		specularLight = clamp(pow(specularLight, shininess), 0.0, 1.0);
	
		float light = ambientLight + (diffuseLight * 1.55) + (specularLight * 0.5);
	
		vec3 grassColorHSV = vec3(0.17 + (genRan(position.xy, 4353.5634534) / 20), 1, 1);
		grassColorHSV = vec3(0.27, 1, 1);
		vec3 grassColorRGB = HSVtoRGB(grassColorHSV);

		vec3 lightColor = vec3(1.0, 0.8, 0.8);
		vec4 color = texture2D(u_grassTex, uv);

		//color.rgb = color.rgb * grassColorRGB;
		if(u_light)
		  color.rgb = light * color.rgb * grassColorRGB;
		else
			color.rgb = color.rgb * vec3(0.15,1,0.15);

    //color.rgb = color.rgb * intensity;
    float alpha = color.a;//texture2D(u_grassAlpha, uv);
		//if(alpha.x < 0.1)
		//   color.rgb = vec3(0,0,0);
    //FragColor = vec4(uv.x, uv.y, 0, alpha);
    //FragColor = vec4(vsColor.rgb, 1);
    //FragColor = vec4(color.rgb, alpha > 0.1 ? 1 : 0);
		gl_FragData[0] = vec4(color.rgb, alpha > 0.1 ? 1 : 0);
    //gl_FragData[1] = vec4(screenPos.z);
    //FragColor = vec4(vec3(1,0,0), alpha > 0.1 ? 1 : 0);
    //FragColor = vec4(uv.x, uv.y, 0, alpha);
    //FragColor = vec4(1,0,0,1);
}