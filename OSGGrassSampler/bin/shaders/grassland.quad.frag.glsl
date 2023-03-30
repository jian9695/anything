#version 120
varying vec2 texcoord;
varying vec3 position;
uniform sampler2D u_grassTex;
uniform sampler2D u_grassAlpha;
uniform float top;
uniform float left;
uniform float resolution;
uniform vec3 cameraPosition;

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
    float t = -dot(p,nor)/dot(rd,nor);
    if( t<0.0 ) return vec3(-1.0);
    
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

#if 0
    // Cramer's rule for solcing p(t) = ro+t·rd = p(u,v) = vo + u·(v1-v0) + v·(v2-v1)
    float d = 1.0/determinant(mat3(v1v0, v2v0, -rd ));
    float u =   d*determinant(mat3(rov0, v2v0, -rd ));
    float v =   d*determinant(mat3(v1v0, rov0, -rd ));
    float t =   d*determinant(mat3(v1v0, v2v0, rov0));
#else
    // The four determinants above have lots of terms in common. Knowing the changing
    // the order of the columns/rows doesn't change the volume/determinant, and that
    // the volume is dot(cross(a,b,c)), we can precompute some common terms and reduce
    // it all to:
    vec3  n = cross( v1v0, v2v0 );
    vec3  q = cross( rov0, rd );
    float d = 1.0/dot( rd, n );
    float u = d*dot( -q, v2v0 );
    float v = d*dot(  q, v1v0 );
    float t = d*dot( -n, rov0 );
#endif    

    if( u<0.0 || v<0.0 || (u+v)>1.0 ) t = -1.0;
    
    return vec3( t, u, v );
}

void main(void)
{ 
   vec2 index = floor(vec2((position.x - left) / resolution, (top - position.y) / resolution));
   float quadHalf = resolution * 0.25;
   vec3 rayDir = normalize(position - cameraPosition);

   int steps = 1;
   vec3 intersect = vec3(-1, -1, -1);
   for(int y=-steps;y<=steps;y++)
   {
      for(int x=-steps;x<=steps;x++)
      {
          vec2 center;
          center.x = left + (index.x + x) * resolution + resolution * 0.5;
          center.y = top - (index.y + y) * resolution - resolution * 0.5;
	        vec3 v0 = vec3(center.x-quadHalf, center.y, quadHalf*2);
	        vec3 v1 = vec3(center.x-quadHalf, center.y, 0);
	        vec3 v2 = vec3(center.x+quadHalf, center.y, 0);
	        vec3 v3 = vec3(center.x+quadHalf, center.y, quadHalf*2);
          vec3 result = quadIntersect(cameraPosition, rayDir, v0, v1, v2, v3);
          if(result.x > 0.0 && (intersect.x < 0.0 || result.x < intersect.x))
          {
             intersect = result;
          }
      }
   }

   vec2 uv = fract(vec2((position.x - left) / resolution, (top - position.y) / resolution));
   if (intersect.x <= 0.0)
   {
      gl_FragColor = vec4(uv.x*0.8,uv.y*0.8,0.5,1);

      return;
   }
   gl_FragColor = vec4(intersect.y,intersect.z,0.15,1);
   //gl_FragColor = vec4(rayDir,1);
   //vec4 color = texture2D(u_grassTex, uv);
   //vec4 alpha = texture2D(u_grassAlpha, uv);
   //gl_FragColor = vec4(color.rgb * vec3(0.15,1,0.15), alpha.x);
}