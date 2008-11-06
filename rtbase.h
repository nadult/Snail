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

Matrix<Vec4f> Inverse(const Matrix<Vec4f>&);


template <class A,class B> struct TIsSame { enum { value=0 }; };
template<class A> struct TIsSame<A,A> { enum { value=1 }; };

template <class A,class B,bool v> struct TSwitch { typedef A Result; };
template <class A,class B> struct TSwitch<A,B,false> { typedef B Result; };

enum IntersectionFlags {
	ifDistance	=1,
	ifElement	=2,
	ifObject	=4,
};

template <class real,int packetSize,int flags_>
class Intersection /*: public Intersection2<real,packetSize,flags_>*/ {
public:
	typedef typename TSwitch<i32x4,u32,TIsSame<real,f32x4>::value>::Result integer;
	typedef typename TSwitch<Vec3q,Vec3f,TIsSame<real,f32x4>::value>::Result Vec;
	enum { flags=flags_ };

private:
	real    distance[flags&ifDistance?packetSize:0];
	integer	object[flags&ifObject?packetSize:0];
	integer	element[flags&ifElement?packetSize:0];

public:
	real &Distance(int q=0) {
	//	static_assert(flags&ifDistance,"Structure doesnt contain 'distance' member.");
		return distance[q];
	}
	const real &Distance(int q=0) const {
	//	static_assert(flags&ifDistance,"Structure doesnt contain 'distance' member.");
		return distance[q];
	}
	integer &Object(int q=0) {
	//	static_assert(flags&ifObject,"Structure doesnt contain 'object' member.");
		return object[q];
	}
	const integer &Object(int q=0) const {
	//	static_assert(flags&ifObject,"Structure doesnt contain 'object' member.");
		return object[q];
	}
	integer &Element(int q=0) {
	//	static_assert(flags&ifElement,"Structure doesnt contain 'element' member.");
		return element[q];
	}
	const integer &Element(int q=0) const {
	//	static_assert(flags&ifElement,"Structure doesnt contain 'element' member.");
		return element[q];
	}

	template <int tflags>
	void Insert(const Intersection<real,packetSize/4,tflags> &rhs,int pos) {
		enum { cflags=tflags&flags };
		if(cflags&ifDistance) for(int q=0;q<packetSize/4;q++) distance[q+pos*(packetSize/4)]=rhs.Distance(q);
		if(cflags&ifObject  ) for(int q=0;q<packetSize/4;q++) object  [q+pos*(packetSize/4)]=rhs.Object  (q);
		if(cflags&ifElement ) for(int q=0;q<packetSize/4;q++) element [q+pos*(packetSize/4)]=rhs.Element (q);
	}

	template <int tflags>
	const Intersection &operator=(const Intersection<real,packetSize,tflags> &rhs) {
		enum { cflags=tflags&flags };
		if(cflags&ifDistance) for(int q=0;q<packetSize;q++) distance[q]=rhs.distance(q);
		if(cflags&ifObject  ) for(int q=0;q<packetSize;q++) object  [q]=rhs.object  (q);
		if(cflags&ifElement ) for(int q=0;q<packetSize;q++) element [q]=rhs.element (q);
	}
};

template<>
class Intersection<float,1,ifDistance> {
public:
	enum { flags=ifDistance };
	typedef u32 integer;
	typedef Vec3f Vec;

private:
	float distance[1];

public:
	float &Distance(int) { return distance[0]; }
	const float &Distance(int) const { return distance[0]; }
	u32 &Object(int) { throw 0; }
	const u32 &Object(int) const { throw 0; }
	u32 &Element(int) { throw 0; }
	const u32 &Element(int) const { throw 0; }
	

	operator float() const { return distance[0]; }
};

#include "bounding_box.h"


#endif

