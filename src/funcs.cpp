#include "rtbase.h"
#include <libgen.h>
#include <errno.h>
#include <cstring>
#include <fwk/gfx/font.h>
#include <fwk/gfx/gl_texture.h>

float BoxPointDistanceSq(const BBox &box,const Vec3f &point) {
	float xMin=box.min.x,yMin=box.min.y,zMin=box.min.z;
	float xMax=box.max.x,yMax=box.max.y,zMax=box.max.z;

//	Vec3f closest;
	float sqrDist=0.0f;
	float delta;
	
	if ( point.x < xMin ) {
		delta = point.x - xMin;
		sqrDist += delta*delta;
	//	closest.x = xMin;
	}
	else if ( point.x > xMax ) {
		delta = point.x - xMax;
		sqrDist += delta*delta;
	//	closest.x = xMax;
	}
//	else closest.x=point.x;

	if ( point.y < yMin ) {
		delta = point.y - yMin;
		sqrDist += delta*delta;
	//	closest.y = yMin;
	}
	else if ( point.y > yMax ) {
		delta = point.y - yMax;
		sqrDist += delta*delta;
	//	closest.y = yMax;
	}
//	else closest.y=point.y;

	if ( point.z < zMin ) {
		delta = point.z - zMin;
		sqrDist += delta*delta;
	//	closest.z = zMin;
	}
	else if ( point.z > zMax ) {
		delta = point.z - zMax;
		sqrDist += delta*delta;
	//	closest.z = zMax;
	}
//	else closest.z=point.z;

	return sqrDist;
}

Matrix<Vec4f> Inverse(const Matrix<Vec4f> &mat) {
	Matrix<Vec4f> mOut;
	
	const float *M=&mat.x.x;
	float *out=&mOut.x.x;
	
	float t[12],m[16];
	for(int n=0;n<4;n++) {
	  m[n+ 0]=M[n*4+0]; m[n+ 4]=M[n*4+1];
	  m[n+ 8]=M[n*4+2]; m[n+12]=M[n*4+3];
   }
	t[0 ]=m[10]*m[15]; t[1 ]=m[11]*m[14];
	t[2 ]=m[9 ]*m[15]; t[3 ]=m[11]*m[13];
	t[4 ]=m[9 ]*m[14]; t[5 ]=m[10]*m[13];
	t[6 ]=m[8 ]*m[15]; t[7 ]=m[11]*m[12];
	t[8 ]=m[8 ]*m[14]; t[9 ]=m[10]*m[12];
	t[10]=m[8 ]*m[13]; t[11]=m[9 ]*m[12];

	out[0 ] = t[0 ]*m[5 ]+t[3 ]*m[6 ]+t[4 ]*m[7 ];
	out[0 ]-= t[1 ]*m[5 ]+t[2 ]*m[6 ]+t[5 ]*m[7 ];
	out[1 ] = t[1 ]*m[4 ]+t[6 ]*m[6 ]+t[9 ]*m[7 ];
	out[1 ]-= t[0 ]*m[4 ]+t[7 ]*m[6 ]+t[8 ]*m[7 ];
	out[2 ] = t[2 ]*m[4 ]+t[7 ]*m[5 ]+t[10]*m[7 ];
	out[2 ]-= t[3 ]*m[4 ]+t[6 ]*m[5 ]+t[11]*m[7 ];
	out[3 ] = t[5 ]*m[4 ]+t[8 ]*m[5 ]+t[11]*m[6 ];
	out[3 ]-= t[4 ]*m[4 ]+t[9 ]*m[5 ]+t[10]*m[6 ];
	out[4 ] = t[1 ]*m[1 ]+t[2 ]*m[2 ]+t[5 ]*m[3 ];
	out[4 ]-= t[0 ]*m[1 ]+t[3 ]*m[2 ]+t[4 ]*m[3 ];
	out[5 ] = t[0 ]*m[0 ]+t[7 ]*m[2 ]+t[8 ]*m[3 ];
	out[5 ]-= t[1 ]*m[0 ]+t[6 ]*m[2 ]+t[9 ]*m[3 ];
	out[6 ] = t[3 ]*m[0 ]+t[6 ]*m[1 ]+t[11]*m[3 ];
	out[6 ]-= t[2 ]*m[0 ]+t[7 ]*m[1 ]+t[10]*m[3 ];
	out[7 ] = t[4 ]*m[0 ]+t[9 ]*m[1 ]+t[10]*m[2 ];
	out[7 ]-= t[5 ]*m[0 ]+t[8 ]*m[1 ]+t[11]*m[2 ];

	t[0 ]=m[2 ]*m[7 ]; t[1 ]=m[3 ]*m[6 ];
	t[2 ]=m[1 ]*m[7 ]; t[3 ]=m[3 ]*m[5 ];
	t[4 ]=m[1 ]*m[6 ]; t[5 ]=m[2 ]*m[5 ];
	t[6 ]=m[0 ]*m[7 ]; t[7 ]=m[3 ]*m[4 ];
	t[8 ]=m[0 ]*m[6 ]; t[9 ]=m[2 ]*m[4 ];
	t[10]=m[0 ]*m[5 ]; t[11]=m[1 ]*m[4 ];

	out[8 ] = t[0 ]*m[13]+t[3 ]*m[14]+t[4 ]*m[15];
	out[8 ]-= t[1 ]*m[13]+t[2 ]*m[14]+t[5 ]*m[15];
	out[9 ] = t[1 ]*m[12]+t[6 ]*m[14]+t[9 ]*m[15];
	out[9 ]-= t[0 ]*m[12]+t[7 ]*m[14]+t[8 ]*m[15];
	out[10] = t[2 ]*m[12]+t[7 ]*m[13]+t[10]*m[15];
	out[10]-= t[3 ]*m[12]+t[6 ]*m[13]+t[11]*m[15];
	out[11] = t[5 ]*m[12]+t[8 ]*m[13]+t[11]*m[14];
	out[11]-= t[4 ]*m[12]+t[9 ]*m[13]+t[10]*m[14];
	out[12] = t[2 ]*m[10]+t[5 ]*m[11]+t[1 ]*m[9 ];
	out[12]-= t[4 ]*m[11]+t[0 ]*m[9 ]+t[3 ]*m[10];
	out[13] = t[8 ]*m[11]+t[0 ]*m[8 ]+t[7 ]*m[10];
	out[13]-= t[6 ]*m[10]+t[9 ]*m[11]+t[1 ]*m[8 ];
	out[14] = t[6 ]*m[9 ]+t[11]*m[11]+t[3 ]*m[8 ];
	out[14]-= t[10]*m[11]+t[2 ]*m[8 ]+t[7 ]*m[9 ];
	out[15] = t[10]*m[10]+t[4 ]*m[8 ]+t[9 ]*m[9 ];
	out[15]-= t[8 ]*m[9 ]+t[11]*m[10]+t[5 ]*m[8 ];

	float det=1.0f/(m[0]*out[0]+m[1]*out[1]+m[2]*out[2]+m[3]*out[3]);
	for(int n=0;n<16;n++) out[n]*=det;
   
	return mOut;   
}

fwk::Ex<fwk::Font> loadFont(const string &name) {
	auto core = EXPECT_PASS(fwk::FontCore::load("data/fonts/" + name + ".fnt"));
	auto tex = EXPECT_PASS(fwk::GlTexture::load("data/fonts/" + core.textureName()));
	return fwk::Font{std::move(core), std::move(tex)};
}

