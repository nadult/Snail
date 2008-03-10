#ifndef RTBASE_H
#define RTBASE_H

#include <cassert>
#include <vector>
#include <exception>
#include <string>
#include <memory.h>
#include "omp.h"
#include "veclib.h"


using namespace veclib;

typedef Vec2<float> Vec2f;
typedef Vec3<float> Vec3f;
typedef Vec4<float> Vec4f;
typedef SSEPVec2	Vec2p;
typedef SSEPVec3	Vec3p;
typedef SSEPVec4	Vec4p;
typedef SSEVec2		Vec2q;
typedef SSEVec3		Vec3q;
typedef SSEVec4		Vec4q;
typedef SSEReal		floatq;
typedef SSEI32		intq;

using std::vector;
using std::pair;
using std::string;

class Exception: public std::exception
{
	string data;
public:
	Exception(const string &txt) :data(txt) { }
	Exception(const char *txt) :data(txt) { }
	~Exception() throw() { }
	const char *what() const throw() { return data.c_str(); }
};

#endif

