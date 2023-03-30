#version 120
varying vec2 texcoord;
varying vec3 position;
uniform sampler2D u_grassTex;
uniform sampler2D u_grassAlpha;
uniform float top;
uniform float left;
uniform float right;
uniform float bottom;
uniform bool showGrid;
float resolution;
uniform vec3 cameraPosition;
const float kHalfPi = 1.5707;
const float kPi = 3.1415926;

float cross2d( in vec2 a, in vec2 b ) { return a.x*b.y - a.y*b.x; }

const int lut[4] = int[](1,2,0,1);

// 0--b--3
// |\
// a c
// |  \
// 1    2
//
vec3 quadIntersect(in vec3 ro, in vec3 rd, in vec3 v0, in vec3 v1, in vec3 v2, in vec3 v3)
{
    // lets make v0 the origin
    vec3 a = v1 - v0;
    vec3 b = v3 - v0;
    vec3 c = v2 - v0;
    vec3 p = ro - v0;

    // intersect plane
    vec3 nor = cross(a,b);
    float temp = dot(rd,nor);
    float t = -dot(p,nor)/temp;
    if( t<0.0 || temp == 0.0) 
       return vec3(-1.0);
    
    // intersection point
    vec3 pos = p + t*rd;

    // see here: https://www.shadertoy.com/view/lsBSDm
    
    // select projection plane
    vec3 mor = abs(nor);
    int id = (mor.x>mor.y && mor.x>mor.z ) ? 0 : 
             (mor.y>mor.z)                 ? 1 : 
                                             2 ;

    int idu = lut[id  ];
    int idv = lut[id+1];
    
    // project to 2D
    vec2 kp = vec2( pos[idu], pos[idv] );
    vec2 ka = vec2( a[idu], a[idv] );
    vec2 kb = vec2( b[idu], b[idv] );
    vec2 kc = vec2( c[idu], c[idv] );
    
    // find barycentric coords of the quadrilateral
    vec2 kg = kc-kb-ka;

    float k0 = cross2d( kp, kb );
    float k2 = cross2d( kc-kb, ka );        // float k2 = cross2d( kg, ka );
    float k1 = cross2d( kp, kg ) - nor[id]; // float k1 = cross2d( kb, ka ) + cross2d( kp, kg );
    
    // if edges are parallel, this is a linear equation
	float u, v;
    if( abs(k2)<0.00001 )
    {
		v = -k0/k1;
        u = cross2d( kp, ka )/k1;
    }
	else
    {
        // otherwise, it's a quadratic
        float w = k1*k1 - 4.0*k0*k2;
        if( w<0.0 ) return vec3(-1.0);
        w = sqrt( w );

        float ik2 = 1.0/(2.0*k2);

                             v = (-k1 - w)*ik2; 
        if( v<0.0 || v>1.0 ) v = (-k1 + w)*ik2;
        
        u = (kp.x - ka.x*v)/(kb.x + kg.x*v);
    }
    
    if( u<0.0 || u>1.0 || v<0.0 || v>1.0) return vec3(-1.0);
    
    return vec3( t, u, v );
}

//=====================================================

float sphIntersect( in vec3 ro, in vec3 rd, in vec4 sph )
{
	vec3 oc = ro - sph.xyz;
	float b = dot( oc, rd );
	float c = dot( oc, oc ) - sph.w*sph.w;
	float h = b*b - c;
	if( h<0.0 ) return -1.0;
	return -b - sqrt( h );
}

// Triangle intersection. Returns { t, u, v }
vec3 triIntersect( in vec3 ro, in vec3 rd, in vec3 v0, in vec3 v1, in vec3 v2 )
{
    vec3 v1v0 = v1 - v0;
    vec3 v2v0 = v2 - v0;
    vec3 rov0 = ro - v0;

    // The four determinants above have lots of terms in common. Knowing the changing
    // the order of the columns/rows doesn't change the volume/determinant, and that
    // the volume is dot(cross(a,b,c)), we can precompute some common terms and reduce
    // it all to:
    vec3  n = cross( v1v0, v2v0 );
    vec3  q = cross( rov0, rd );
    float d = 1.0/dot( rd, n );
    float t = d*dot( -n, rov0 ); 
    if(t < 0.0) 
       return vec3(t, -1, -1);
    float u = d*dot( -q, v2v0 );
    float v = d*dot(  q, v1v0 );

    if( u<0.0 || v<0.0 || (u+v)>1.0 ) 
       t = -1.0;
    
    return vec3( t, u, v );
}

float genRan(vec2 st, float seed) {
    return fract(sin(dot(st.xy + vec2(16.9315,62.924), vec2(12.9898,78.233))) * seed);
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

const float grassPerCell = 10;
const float xSeed[10] = float[](53758.5453123,23758.5453123,33758.5453123,43758.5453123,53758.5453123,63758.5453123,73758.5453123,93758.5453123,93758.5453123,103758.5453123);
const float ySeed[10] = float[](13751.5453123,23752.5453123,33753.5453123,43754.5453123,53755.5453123,63756.5453123,73757.5453123,93718.5453123,93759.5453123,103750.5453123);
const float widthSeed[10] =   float[](11751.5453123,22752.5453123,33753.5454123,44754.5453123,55755.5453123,66756.5453123,77757.5453123,98718.5453123,99759.5453123,90750.5453123);
const float heightSeed[10] =  float[](11151.5453123,22252.5453123,33353.5454123,44454.5453123,55555.5453123,66656.5453123,77757.5453123,98818.5453123,99959.5453123,92150.5453123);
const float rotateSeed[10] =  float[](16218.2443128,26218.2443128,36218.2443128,46218.2443128,56218.2443128,66218.2443128,76218.2443128,86218.2443128,96218.2443128,106218.2443128);
const float tiltSeed[10] =    float[](12471.2257131,22471.527131,32471.5257131,42471.5257131,52471.5257131,62471.5257131,72471.5257131,82471.5257131,92471.5257131,102471.5257131);
const float centerX[10]  =    float[](0.166666667, 0.5,         0.833333333, 0.166666667, 0.5, 0.833333333, 0.166666667, 0.5,         0.833333333, 0.5);
const float centerY[10]  =    float[](0.166666667, 0.166666667, 0.166666667, 0.5,         0.5, 0.5,         0.833333333, 0.833333333, 0.833333333, 0.5);
const float rotates[10] =    float[](0,60,44,270,130,190,335,86,227,310);

vec4 intersectCell(vec2 index, vec3 pos, vec3 rayDir, float riseToRun)
{   
   vec4 intersect = vec4(-1);
   if(index.x < 0 || index.y < 0)
     return intersect;
   float alpha = 0.0;
   vec2 cellCenter;
   cellCenter.x = left + index.x * resolution + resolution * 0.5;
   cellCenter.y = top - index.y * resolution - resolution * 0.5; 
  //cellCenter.x = left + index.x * resolution;
   //cellCenter.y = top - index.y * resolution; 
   if(cellCenter.x > right || cellCenter.y < bottom)
      return vec4(-1);

   for(int i=1;i<=1;i++)
   {
     float seedFactor = i * 2.3435304;
  for(int n=0;n<grassPerCell;n++)
   {
   //vec2 center = cellCenter + vec2(centerX[n], -centerY[n]) * resolution;
   vec2 center = cellCenter;
   float xoffset = genRan(center+vec2(6.3435304,4.3434106), xSeed[n]*seedFactor);
   float yoffset = genRan(center+vec2(3.4753849,6.3435304), ySeed[n]*seedFactor);
   float ranWidth = genRan(center, widthSeed[n]*seedFactor);
   float ranHeight = genRan(center, heightSeed[n]*seedFactor);
   center.x += ((xoffset - 0.5)  * resolution);
   center.y += ((yoffset - 0.5)  * resolution);
   //center.x += ((xoffset - 0.5) * (resolution / 3));
   //center.y += ((yoffset - 0.5) * (resolution / 3));

   float grassHeight = resolution * 0.5 + ranHeight * resolution;
   float grassWidth =  resolution * 0.1 + ranWidth * resolution * 0.05;

   //float grassWidth = 0.5;
   //float grassHeight = 1;
   float riseToRunHere = 1 / length(position - vec3(center.x,center.y,1));
   //if(riseToRunHere < riseToRun)
   //   continue;


   vec3 cameraToObj = cameraPosition - vec3(center.x,center.y,grassHeight*0.5);
   //float angleBetween = acos(dot(rayDir, cameraToObj) / (length(rayDir) * length(cameraToObj)));
   //if(angleBetween > kPi / 4)
    //  continue;
   //if(sphIntersect(cameraPosition, -rayDir, vec4(center.x,center.y,grassHeight*0.5, grassHeight*0.5)) < 0)
    //  continue;
   //float randomRotation = rotates[n] * kPi / 180;//2 * kPi * genRan(center, rotateSeed[n]*seedFactor);
	 float randomRotation = 2 * kPi * genRan(center, rotateSeed[n]*seedFactor);
	 
   float ranTiltFrac = genRan(center, tiltSeed[n]*seedFactor);
	 float randomTilt = kHalfPi * ranTiltFrac;
   mat3 rotationMatrix = createRotationMatrix(vec3(0,0,1), randomRotation);
	 vec3 tiltVec = normalize(rotationMatrix * vec3(0,1,0));
	 tiltVec = cross(tiltVec, vec3(0,0,1));
	 rotationMatrix = createRotationMatrix(tiltVec, randomTilt) * rotationMatrix;

	 vec3 v0 = rotationMatrix * vec3(-grassWidth*0.5, 0, grassHeight) + vec3(center, 0);
	 vec3 v1 = rotationMatrix * vec3(-grassWidth*0.5, 0, 0) + vec3(center, 0);
	 vec3 v2 = rotationMatrix * vec3(grassWidth*0.5,  0, 0) + vec3(center, 0);
	 vec3 v3 = rotationMatrix * vec3(grassWidth*0.5,  0, grassHeight) + vec3(center, 0);
   
   vec3 result = quadIntersect(cameraPosition, -rayDir, v0, v1, v2, v3);
   if(result.x > 0.0)
   {  
      result.z = 1.0 - result.z;
      float texAlpha = texture2D(u_grassAlpha, result.yz).x;
      if(texAlpha > 0.1)
      {
         intersect = vec4(result, texAlpha);
      }
    }
  }

   }

  return intersect;
}

vec4 intersectCell(vec2 index, vec3 rayDir)
{   
   if(index.x < 0 || index.y < 0)
     return vec4(-1);
   float alpha = 0.0;
   vec2 center;
   center.x = left + index.x * resolution + resolution * 0.5;
   center.y = top - index.y * resolution - resolution * 0.5; 
   if(center.x > right || center.y < bottom)
      return vec4(-1);

   float exist = genRan(center, 34558.5453123);
   if(exist < 0.25)
      return vec4(-1);

   float xoffset = genRan(center, 13758.5453123);
   float yoffset = genRan(center, 25672.5643742);
   float ranWidth = genRan(center, 68672.5443847);
   float ranHeight = genRan(center, 45216.6742694);
   //center.x += ((xoffset - 0.5) * 0.75 * resolution);
   //center.y += ((yoffset - 0.5) * 0.75 * resolution);
   float grassWidth = 0.001;// + ranWidth * 0.001;
   float grassHeight = 0.02;// + ranHeight * 0.01;
   //float grassWidth = 0.5;
   //float grassHeight = 1;
   float riseToRunHere = grassHeight / length(position - vec3(center.x,center.y,grassHeight));
   //if(riseToRunHere < riseToRun)
   //  continue;

   float randomRotation = 2 * kPi * genRan(center, 16218.2443128);
	 float ranTiltFrac = genRan(center, 82471.5257131);
	 float randomTilt = kHalfPi * 0.25 * ranTiltFrac;
   //randomRotation = 0;
   mat3 rotationMatrix = createRotationMatrix(vec3(0,0,1), randomRotation);
	 vec3 tiltVec = normalize(rotationMatrix * vec3(0,1,0));
	 tiltVec = cross(tiltVec, vec3(0,0,1));
	 rotationMatrix = createRotationMatrix(tiltVec, randomTilt) * rotationMatrix;

	 vec3 v0 = rotationMatrix * vec3(-grassWidth*0.5, 0, grassHeight) + vec3(center, 0);
	 vec3 v1 = rotationMatrix * vec3(-grassWidth*0.5, 0, 0) + vec3(center, 0);
	 vec3 v2 = rotationMatrix * vec3(grassWidth*0.5,  0, 0) + vec3(center, 0);
	 vec3 v3 = rotationMatrix * vec3(grassWidth*0.5,  0, grassHeight) + vec3(center, 0);

   vec3 result = quadIntersect(cameraPosition, -rayDir, v0, v1, v2, v3);
   if(result.x > 0.0)
   {  
      result.z = 1.0 - result.z;
      float texAlpha = texture2D(u_grassAlpha, result.yz).x;
      if(texAlpha > 0.1)
         return vec4(result, texAlpha);
    }
    return vec4(-1);
}

void main(void)
{ 
   vec3 rayDir = cameraPosition - position;
   float cameraDist = length(rayDir);
   rayDir = normalize(rayDir);
   resolution = 0.003;//max(0.001, cameraPosition.z / 150);
   vec2 index = floor(vec2((position.x - left) / resolution, (top - position.y) / resolution));
   float quadHalf = resolution * 0.25;
   vec2 lastIndex = index;
   int maxSteps = 100;
   float maxGrassHeight = 0.02;
   float riseToRun = cameraPosition.z / cameraDist;
   float cotanRay = length(position - vec3(cameraPosition.x, cameraPosition.y, 0)) / cameraDist;
   float marchDelta = resolution * 0.5;// cotanRay;
   float marchDist = min(maxGrassHeight / riseToRun, cameraDist - 0.01);//maxGrassHeight / riseToRun;
   vec3 marchPos = position + rayDir * marchDist;
   vec3 marchIncrement = rayDir * marchDelta;
   vec3 intersect = vec3(1000000, -1, -1);
   float alpha;
   int steps = 1;
   /*for(int y=-steps;y<=steps;y++)
   {
      for(int x=-steps;x<=steps;x++)
      {
         vec4 result = intersectCell(vec2(index.x+x,index.y+y), position, rayDir, riseToRun);
         if(result.x > 0)
         {
            intersect = result.xyz;
            alpha = result.w;
            maxSteps = 0;
            break;
         }
      }
   }*/

   steps = 0;
   while(steps <= maxSteps)
   {      
      vec2 curIndex = index;
      if(steps > 0)
      {  
        marchDist -= marchDelta;
        if(marchDist < 0)
           break;
      }

      marchPos = position + rayDir * marchDist;
      curIndex = floor(vec2((marchPos.x - left) / resolution, (top - marchPos.y) / resolution));

      if(steps > 0 && abs(curIndex.x - lastIndex.x) < 0.1 && abs(curIndex.y - lastIndex.y) < 0.1)
      {
         continue;
      }
      lastIndex = curIndex;
      
      steps++;
      int win = 1;
      for(int n = -win;n < win;n++)
      {
         for(int m = -win;m < win;m++)
         {
           vec4 result = intersectCell(vec2(curIndex.x+n,curIndex.y+m), rayDir);
           if(result.x > 0 && result.x < intersect.x)
           {
              intersect = result.xyz;
              alpha = result.w;     
              //break;
           }
         }
      }
   }

   vec2 uv = fract(vec2((position.x - left) / resolution, (top - position.y) / resolution));
   if (intersect.x <= 0.0 || intersect.x >= 1000000)
   {
      if(showGrid)
         gl_FragColor = vec4(uv.x*0.8,uv.y*0.8,0.5,1);
      else
         gl_FragColor = vec4(0.5,0.5,0.5,1);
      return;
   }
   vec4 color = texture2D(u_grassTex, intersect.yz);
   gl_FragColor = vec4(color.rgb * vec3(0.15,1,0.15), alpha);
   //gl_FragColor = vec4(intersect.y, intersect.z, 0.2, alpha);
}

          /*vec3 result = triIntersect(cameraPosition, rayDir, v1, v0, v2);
          if(result.x > 0.0 && (intersect.x < 0.0 || result.x < intersect.x))
          {          
             float texAlpha = texture2D(u_grassAlpha, result.yz).x;
             if(texAlpha > 0.5)
             {
                intersect = result;
                alpha = texAlpha;
             }
          }

          result = triIntersect(cameraPosition, rayDir, v2, v0, v3);
          if(result.x > 0.0 && (intersect.x < 0.0 || result.x < intersect.x))
          {          
             float texAlpha = texture2D(u_grassAlpha, result.yz).x;
             if(texAlpha > 0.5)
             {
                intersect = result;
                alpha = texAlpha;
             }
          }*/
