#ifndef RTBASE_H
#define RTBASE_H

#define VECLIB_SSE_VER 0x20

#include <baselib.h>
#include <veclib.h>
#include "allocator.h"
#include <cassert>
#include <math.h>


//#define EXPECT_TAKEN(a)         __builtin_expect(!!(a), 1)
//#define EXPECT_NOT_TAKEN(a)   	__builtin_expect(!!(a), 0)
#define EXPECT_TAKEN(a)			a
#define EXPECT_NOT_TAKEN(a)		a

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

INLINE bool isnan(float t) {
	union { float f; int i; }; f=t;
	return (i&0x7f800000)==0x7f8 && (i&0x7fffff)!=0;
}

inline float Maximize(const floatq &t) { return Max(Max(t[0],t[1]),Max(t[2],t[3])); }
inline float Minimize(const floatq &t) { return Min(Min(t[0],t[1]),Min(t[2],t[3])); }
inline Vec2p Maximize(const Vec2q &v) { return Vec2p(Maximize(v.x),Maximize(v.y)); }
inline Vec2p Minimize(const Vec2q &v) { return Vec2p(Minimize(v.x),Minimize(v.y)); }
inline Vec3p Maximize(const Vec3q &v) { return Vec3p(Maximize(v.x),Maximize(v.y),Maximize(v.z)); }
inline Vec3p Minimize(const Vec3q &v) { return Vec3p(Minimize(v.x),Minimize(v.y),Minimize(v.z)); }

INLINE bool IsNan(const Vec3f &f) { return isnan(f.x)||isnan(f.y)||isnan(f.z); }
INLINE bool IsNan(const Vec4f &f) { return isnan(f.x)||isnan(f.y)||isnan(f.z)||isnan(f.w); }
INLINE bool IsNan(const Vec3q &f) { return 
		isnan(f.x[0])||isnan(f.y[0])||isnan(f.z[0])||
		isnan(f.x[1])||isnan(f.y[1])||isnan(f.z[1])||
		isnan(f.x[2])||isnan(f.y[2])||isnan(f.z[2])||
		isnan(f.x[3])||isnan(f.y[3])||isnan(f.z[3]); }

INLINE Vec3q SafeInv(const Vec3q &v) {
	const float epsilon=0.0000001f,inf=1.0f/0.0f;
	return Vec3q(
		Condition(Abs(v.x)<floatq(epsilon),floatq(inf),Inv(v.x)),
		Condition(Abs(v.y)<floatq(epsilon),floatq(inf),Inv(v.y)),
		Condition(Abs(v.z)<floatq(epsilon),floatq(inf),Inv(v.z)));
}

INLINE Vec3f SafeInv(const Vec3f &v) {
	const float epsilon=0.0000001f,inf=1.0f/0.0f;
	return Vec3f(
		Condition(Abs(v.x)<epsilon,inf,Inv(v.x)),
		Condition(Abs(v.y)<epsilon,inf,Inv(v.y)),
		Condition(Abs(v.z)<epsilon,inf,Inv(v.z)));
}

template <int size>
void ComputeMinMax(const Vec3q *vec,float *__restrict__ min,float *__restrict__ max) {
	floatq tMin[3]={1.0f/0.0f,1.0f/0.0f,1.0f/0.0f};
	floatq tMax[3]={-1.0f/0.0f,-1.0f/0.0f,-1.0f/0.0f};

	for(int q=0;q<size;q++) {
		const Vec3q &v=vec[q];
		tMin[0]=Min(tMin[0],v.x); tMax[0]=Max(tMax[0],v.x);
		tMin[1]=Min(tMin[1],v.y); tMax[1]=Max(tMax[1],v.y);
		tMin[2]=Min(tMin[2],v.z); tMax[2]=Max(tMax[2],v.z);
	}
	
	min[0]=Minimize(tMin[0]); min[1]=Minimize(tMin[1]); min[2]=Minimize(tMin[2]);
	max[0]=Maximize(tMax[0]); max[1]=Maximize(tMax[1]); max[2]=Maximize(tMax[2]);
}

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
		indices[0][last]=idx;
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

template <>
class ObjectIdxBuffer<0>
{
public:
	INLINE void Insert(u32) { }
	INLINE bool Find(u32) { return false; }
};

Matrix<Vec4f> Inverse(const Matrix<Vec4f>&);

#include "tree_stats.h"


template <class A,class B> struct TIsSame { enum { value=0 }; };
template<class A> struct TIsSame<A,A> { enum { value=1 }; };

template <class A,class B,bool v> struct TSwitch { typedef A Result; };
template <class A,class B> struct TSwitch<A,B,false> { typedef B Result; };

namespace isct {

	enum Flags {
		// Isct flags:
		fDistance	= 0x01,
		fElement	= 0x02,
		fObject		= 0x04,
		fStats		= 0x08,

		// Isct & RayGroup flags:
		fPrimary	= 0x100,
		fShadow		= 0x200,	// fObject, fElement will be excluded

		// RayGroup flags:
		fMaxDist	= 0x010000,
		fShOrig		= 0x020000,
	};

}

/*
inline bool TestForNans(const Vec3q &v,int id,bool thr=1) {
	bool nan=isnan(v.x[0])||isnan(v.x[1])||isnan(v.x[2])||isnan(v.x[3]);
	nan=nan||isnan(v.y[0])||isnan(v.y[1])||isnan(v.y[2])||isnan(v.y[3]);
	nan=nan||isnan(v.z[0])||isnan(v.z[1])||isnan(v.z[2])||isnan(v.z[3]);

	if(nan) {
		printf("%d: %f %f %f %f\n%f %f %f %f\n%f %f %f %f\n",id,
				v.x[0],v.x[1],v.x[2],v.x[3],
				v.y[0],v.y[1],v.y[2],v.y[3],
				v.z[0],v.z[1],v.z[2],v.z[3]);
		if(thr) ThrowException("NANs! run for your lives!");
		return 1;
	}

	return 0;
}*/

// Intersection
template <class Real,int packetSize,int flags_>
class Isct {
	enum { flags_0=(flags_&isct::fShadow?flags_&0xff09:flags_)&0xffff };

public:
	enum { flags=flags_0 };
	typedef typename TSwitch<i32x4,u32,TIsSame<Real,f32x4>::value>::Result Int;
	typedef typename TSwitch<Vec3q,Vec3f,TIsSame<Real,f32x4>::value>::Result Vec3;
	typedef typename TSwitch<Vec2q,Vec2f,TIsSame<Real,f32x4>::value>::Result Vec2;

private:
	Real    distance[flags&isct::fDistance?packetSize:0];
	Int		object  [flags&isct::fObject  ?packetSize:0];
	Int		element [flags&isct::fElement ?packetSize:0];

	typedef TreeStats<flags&isct::fStats?1:0> TStats;
	TStats stats;

public:
	int lastShadowTri;
	//Dont worry about those if's, they will be optimized out
	
	Isct() { }

	Real &Distance(int q=0) {
		if(!(flags&isct::fDistance)) ThrowException("Structure doesnt contain 'distance' member.");
		return distance[q];
	}
	const Real &Distance(int q=0) const {
		if(!(flags&isct::fDistance)) ThrowException("Structure doesnt contain 'distance' member.");
		return distance[q];
	}
	Int &Object(int q=0) {
		if(!(flags&isct::fObject)) ThrowException("Structure doesnt contain 'object' member.");
		return object[q];
	}
	const Int &Object(int q=0) const {
		if(!(flags&isct::fObject)) ThrowException("Structure doesnt contain 'object' member.");
		return object[q];
	}
	Int &Element(int q=0) {
		if(!(flags&isct::fElement)) ThrowException("Structure doesnt contain 'element' member.");
		return element[q];
	}
	const Int &Element(int q=0) const {
		if(!(flags&isct::fElement)) ThrowException("Structure doesnt contain 'element' member.");
		return element[q];
	}

	const int LastShadowTri() const { return lastShadowTri; }
	int &LastShadowTri() { return lastShadowTri; } 

	TStats &Stats() { return stats; }
	const TStats &Stats() const { return stats; }

	template <int tflags>
	void Insert(const Isct<Real,packetSize/4,tflags> &rhs,int pos) {
		enum { cflags=tflags&flags };
		if(cflags&isct::fDistance) for(int q=0;q<packetSize/4;q++) distance[q+pos*(packetSize/4)]=rhs.Distance(q);
		if(cflags&isct::fObject  ) for(int q=0;q<packetSize/4;q++) object  [q+pos*(packetSize/4)]=rhs.Object  (q);
		if(cflags&isct::fElement ) for(int q=0;q<packetSize/4;q++) element [q+pos*(packetSize/4)]=rhs.Element (q);
		if(cflags&isct::fStats   ) stats+=rhs.Stats();
		lastShadowTri=rhs.LastShadowTri();
	}

	// Works for Isct<f32x4,1,..>
	template <int tflags>
	void Insert(const Isct<float,1,tflags> &rhs,int pos) {
		enum { cflags=tflags&flags };
		if(cflags&isct::fDistance) distance[0][pos]=rhs.Distance(0);
		if(cflags&isct::fObject  ) object  [0][pos]=rhs.Object  (0);
		if(cflags&isct::fElement ) element [0][pos]=rhs.Element (0);
		if(cflags&isct::fStats   ) stats+=rhs.Stats();
		lastShadowTri=rhs.LastShadowTri();
	}

	template <int tflags>
	Isct(const Isct<Real,packetSize,tflags> &rhs) {
		enum { cflags=tflags&flags };

		if(cflags&isct::fDistance) for(int q=0;q<packetSize;q++) distance[q]=rhs.Distance(q);
		if(cflags&isct::fObject  ) for(int q=0;q<packetSize;q++) object  [q]=rhs.Object  (q);
		if(cflags&isct::fElement ) for(int q=0;q<packetSize;q++) element [q]=rhs.Element (q);
		if(cflags&isct::fStats   ) stats=rhs.Stats();
		lastShadowTri=rhs.LastShadowTri();
	}

	template <int tflags>
	const Isct &operator=(const Isct<Real,packetSize,tflags> &rhs) {
		enum { cflags=tflags&flags };
		if(cflags&isct::fDistance) for(int q=0;q<packetSize;q++) distance[q]=rhs.Distance(q);
		if(cflags&isct::fObject  ) for(int q=0;q<packetSize;q++) object  [q]=rhs.Object  (q);
		if(cflags&isct::fElement ) for(int q=0;q<packetSize;q++) element [q]=rhs.Element (q);
		if(cflags&isct::fStats   ) stats=rhs.Stats();
		lastShadowTri=rhs.LastShadowTri();
		return *this;
	}
};


#include "bounding_box.h"


#endif

