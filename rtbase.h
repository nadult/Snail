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

#endif

