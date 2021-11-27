#ifndef RTBASE_MATH_H
#define RTBASE_MATH_H


#include <veclib.h>
#include <math.h>
#include <fwk/sys_base.h>

#define SERIALIZE_AS_POD(Type) template <> static constexpr bool fwk::is_flat_data<Type> = true;

using namespace veclib;

typedef Vec2<float> Vec2f;
typedef Vec3<float> Vec3f;
typedef Vec4<float> Vec4f;
typedef vec2f32x4	Vec2q;
typedef vec3f32x4	Vec3q;
typedef vec4f32x4	Vec4q;
typedef f32x4		floatq;
typedef i32x4		intq;

SERIALIZE_AS_POD(Vec2f)
SERIALIZE_AS_POD(Vec3f)
SERIALIZE_AS_POD(Vec4f)
SERIALIZE_AS_POD(Vec2q)
SERIALIZE_AS_POD(Vec3q)
SERIALIZE_AS_POD(Vec4q)
SERIALIZE_AS_POD(floatq)
SERIALIZE_AS_POD(intq)

enum {
	blockWidth = 16,
	blockHeight = 64,
};


//TODO: WTF???
#if !defined(__PPC) && !defined(__PPC__)
	#ifdef __BIG_ENDIAN
		#undef __BIG_ENDIAN
	#endif
#endif


#define EXPECT_TAKEN(a)         __builtin_expect(!!(a), 1)
#define EXPECT_NOT_TAKEN(a)   	__builtin_expect(!!(a), 0)
//#define EXPECT_TAKEN(a)			a
//#define EXPECT_NOT_TAKEN(a)		a

#define NOINLINE __attribute__((noinline))



template <class Vec>
const Vec Reflect(const Vec ray,const Vec nrm) {
	typename Vec::TScalar dot = (nrm | ray);
	return ray - nrm * (dot + dot);
}

template <class Vec>
const Vec Normalize(const Vec v) { return v * RSqrt(v | v); }

inline float Maximize(const floatq t) { return Max(Max(t[0], t[1]), Max(t[2], t[3])); }
inline float Minimize(const floatq t) { return Min(Min(t[0], t[1]), Min(t[2], t[3])); }
inline Vec2f Maximize(const Vec2q v) { return Vec2f(Maximize(v.x), Maximize(v.y)); }
inline Vec2f Minimize(const Vec2q v) { return Vec2f(Minimize(v.x), Minimize(v.y)); }
inline Vec3f Maximize(const Vec3q v) { return Vec3f(Maximize(v.x), Maximize(v.y), Maximize(v.z)); }
inline Vec3f Minimize(const Vec3q v) { return Vec3f(Minimize(v.x), Minimize(v.y), Minimize(v.z)); }

inline Vec3f ExtractN(const Vec3q v, int n) { return Vec3f(v.x[n], v.y[n], v.z[n]); }


#endif
