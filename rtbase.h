#ifndef RTBASE_H
#define RTBASE_H

#include <cassert>
#include <vector>
#include <exception>
#include <string>
#include <memory.h>
#include "omp.h"
#include "baselib.h"
#include "veclib.h"


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

using std::pair;

#endif

