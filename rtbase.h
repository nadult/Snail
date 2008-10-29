#ifndef RTBASE_H
#define RTBASE_H

#ifdef _WIN32
	#define VECLIB_SSE_VER 0x20
#endif

#include <baselib.h>
#include <veclib.h>
#include "allocator.h"
#include <cassert>


using namespace baselib;
using namespace veclib;

typedef Vec2<float> Vec2f;
typedef Vec3<float> Vec3f;
typedef Vec4<float> Vec4f;
typedef pvec2f32	Vec2p;
typedef pvec3f32	Vec3p;
typedef pvec4f32	Vec4p;
typedef vec2f32x4	Vec2q;
typedef vec3f32x4	Vec3q;
typedef vec4f32x4	Vec4q;
typedef f32x4		floatq;
typedef i32x4		intq;

template <class Vec>
Vec Reflect(const Vec &ray,const Vec &nrm) {
	typename Vec::TScalar dot=(nrm|ray);
	return ray-nrm*(dot+dot);
}

inline float Maximize(const floatq &t) { return Max(Max(t[0],t[1]),Max(t[2],t[3])); }
inline float Minimize(const floatq &t) { return Min(Min(t[0],t[1]),Min(t[2],t[3])); }
inline Vec3p Maximize(const Vec3q &v) { return Vec3p(Maximize(v.x),Maximize(v.y),Maximize(v.z)); }
inline Vec3p Minimize(const Vec3q &v) { return Vec3p(Minimize(v.x),Minimize(v.y),Minimize(v.z)); }

extern int gVals[16];

template <int size>
class ObjectIdxBuffer
{
public:
	ObjectIdxBuffer() {
		for(int n=0;n<size;n++)
			indices[n]=i32x4(-1);
		last=0;
	}
	void Insert(u32 idx) {
		((u32*)indices)[last]=idx;
		last=(last+1)%(size*4);
	}
	bool Find(u32 idx) {
		i32x4 tidx(idx); i32x4b test;
		test=indices[0]==tidx;
		for(int n=1;n<size;n++)
			test=test||indices[n]==tidx;
		return ForAny(test);
	}
	i32x4 indices[size];
	uint last;
};

Matrix<Vec4f> Inverse(const Matrix<Vec4f>&);



class BBox {
public:
	BBox() { }
	BBox(const Vec3f &tMin,const Vec3f &tMax) :min(tMin),max(tMax) { }
	const BBox &operator*=(const Matrix<Vec4f> &m) {
		Vec3f v[8];
		v[0].x=v[2].x=v[4].x=v[6].x=min.x; v[1].x=v[3].x=v[5].x=v[7].x=max.x;
		v[0].y=v[1].y=v[4].y=v[5].y=min.y; v[3].y=v[2].y=v[6].y=v[7].y=max.y;
		v[0].z=v[1].z=v[2].z=v[3].z=min.z; v[4].z=v[5].z=v[6].z=v[7].z=max.z;

		const Vec3f &tv=v[0];
		min.x=max.x=m.x.x*tv.x+m.y.x*tv.y+m.z.x*tv.z;
		min.y=max.y=m.x.y*tv.x+m.y.y*tv.y+m.z.y*tv.z;
		min.z=max.z=m.x.z*tv.x+m.y.z*tv.y+m.z.z*tv.z;

		for(int n=1;n<8;n++) {
			const Vec3f &tv=v[n];
			float tx=m.x.x*tv.x+m.y.x*tv.y+m.z.x*tv.z;
			float ty=m.x.y*tv.x+m.y.y*tv.y+m.z.y*tv.z;
			float tz=m.x.z*tv.x+m.y.z*tv.y+m.z.z*tv.z;
			if(tx<min.x) min.x=tx; else if(tx>max.x) max.x=tx;
			if(ty<min.y) min.y=ty; else if(ty>max.y) max.y=ty;
			if(tz<min.z) min.z=tz; else if(tz>max.z) max.z=tz;
		}

		min.x+=m.w.x; max.x+=m.w.x;
		min.y+=m.w.y; max.y+=m.w.y;
		min.z+=m.w.z; max.z+=m.w.z;
			
		return *this;
	}
	const BBox &operator+=(const BBox &other) {
		min=VMin(min,other.min);
		max=VMax(max,other.max);
	}
	Vec3f Size() const { return max-min; }
	Vec3f Center() const { return (max+min)*0.5f; }
	
	Vec3f min,max;
};

inline BBox operator+(const BBox &a,const BBox &b) { BBox out(a); out+=b; return out; }
inline BBox operator*(const BBox &a,const Matrix<Vec4f> &mat) { BBox out(a); out*=mat; return out; }



#endif

