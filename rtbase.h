#ifndef RTBASE_H
#define RTBASE_H

#include <fwk/sys_base.h>
#include <fwk/format.h>
#include <fwk/sys/stream.h>
#include "allocator.h"
#include <cassert>
#include <vector>

#include "rtbase_math.h"

#define FATAL FWK_FATAL

using fwk::Stream;
using fwk::string;
using fwk::vector;
using fwk::shared_ptr;
using fwk::make_shared;
using fwk::ErrorChunk;
using fwk::Error;

class MipmapTexture;
using PMipmapTexture = shared_ptr<MipmapTexture>;

namespace fwk {
	class Font;
}

// TODO: these shouldn't be required
template <class T, class A> void loadFromStream(std::vector<T, A> &v, Stream &sr) {
	// TODO: handle errors properly
	u32 size;
	sr.loadData(&size, sizeof(size));
	v.resize(size);

	if(fwk::SerializeAsPod<T>::value)
		sr.loadData(&v[0], sizeof(T) * size);
	else
		for(u32 n = 0; n < size; n++)
			loadFromStream(v[n], sr);
}

template <class T, class A> void saveToStream(const std::vector<T, A> &v, Stream &sr) {
	u32 size;
	size = u32(v.size());
	ASSERT(size_t(size) == v.size());
	sr.saveData(&size, sizeof(size));

	if(fwk::SerializeAsPod<T>::value)
		sr.saveData(&v[0], sizeof(T) * size);
	else
		for(u32 n = 0; n < size; n++)
			saveToStream(v[n], sr);
}


extern int gVals[16];

void FileModTime(const string &path, time_t*);

template <typename T, size_t N>
size_t countof(T (&array)[N]) { return N; }

inline unsigned short ByteSwap(unsigned short v) {
	return ((v & 0xff) << 8) | ((v & 0xff00) >> 8);
}

inline unsigned short ByteSwap(short v) {
	return ((v & 0xff) << 8) | ((v & 0xff00) >> 8);
}

inline double ByteSwap(double v) {
	union { double d; char c[8]; };
	d = v;
	for(int k = 0; k < 4; k++)
		{ char t = c[k]; c[k] = c[7 - k]; c[7 - k] = t; }
	return d;
}

inline int ByteSwap(int i) {
	return __builtin_bswap32(i);
}

inline unsigned int ByteSwap(unsigned int i) {
	return __builtin_bswap32(i);
}

inline float ByteSwap(float i) {
	return BitCast<float>(__builtin_bswap32(BitCast<int>(i)));
}

inline const Vec2f ByteSwap(const Vec2f v)
	{ return Vec2f(ByteSwap(v.x), ByteSwap(v.y)); }

inline const Vec3f ByteSwap(const Vec3f v)
	{ return Vec3f(ByteSwap(v.x), ByteSwap(v.y), ByteSwap(v.z)); }

inline const Vec4f ByteSwap(const Vec4f v)
	{ return Vec4f(ByteSwap(v.x), ByteSwap(v.y), ByteSwap(v.z), ByteSwap(v.w)); }

inline void ByteSwap(unsigned short *v) {
	*v = ((*v & 0xff) << 8) | ((*v & 0xff00) >> 8); //TODO: check for correctness
}

inline void ByteSwap(short *v) {
	*v = ((*v & 0xff) << 8) | ((*v & 0xff00) >> 8);
}

inline void ByteSwap(double *v) {
	union { double d; char c[8]; };
	d = *v;
	for(int k = 0; k < 4; k++)
		{ char t = c[k]; c[k] = c[7 - k]; c[7 - k] = t; }
	*v = d;
}

inline void ByteSwap(int *v) {
	*v = __builtin_bswap32(*v);
}

inline void ByteSwap(unsigned int *v) {
	*v = __builtin_bswap32(*v);
}

inline void ByteSwap(float *v) {
	*v = BitCast<float>(__builtin_bswap32(BitCast<int>(*v)));
}

inline void ByteSwap(Vec2f *v)
	{ *v = Vec2f(ByteSwap(v->x), ByteSwap(v->y)); }

inline void ByteSwap(Vec3f *v)
	{ *v = Vec3f(ByteSwap(v->x), ByteSwap(v->y), ByteSwap(v->z)); }

inline void ByteSwap(Vec4f *v)
	{ *v = Vec4f(ByteSwap(v->x), ByteSwap(v->y), ByteSwap(v->z), ByteSwap(v->w)); }


bool IsNan(const Vec3f f);
bool IsNan(const Vec4f f);
bool IsNan(const Vec3q f);

INLINE Vec3q SafeInv(Vec3q v) {
	//TODO: SafeInv potrzebne np. w modelu angel.obj; Zrobic to jakos szybciej...
	return VInv(v + Vec3q(floatq(0.00000001f)));

/*	const float epsilon=0.0000001f,inf=1.0f/0.0f;
	return Vec3q(
		Condition(Abs(v.x)<floatq(epsilon),floatq(inf),Inv(v.x)),
		Condition(Abs(v.y)<floatq(epsilon),floatq(inf),Inv(v.y)),
		Condition(Abs(v.z)<floatq(epsilon),floatq(inf),Inv(v.z))); */
}

INLINE Vec3f SafeInv(const Vec3f &v) {
	const float epsilon=0.0000001f,inf=1.0f/0.0f;
	return Vec3f(
		Condition(Abs(v.x)<epsilon,inf,Inv(v.x)),
		Condition(Abs(v.y)<epsilon,inf,Inv(v.y)),
		Condition(Abs(v.z)<epsilon,inf,Inv(v.z)));
}

inline int MaxAxis(Vec3f vec) {
	return vec.y > vec.x? vec.z > vec.y? 2 : 1 : vec.z > vec.x? 2 : 0;
}

void ComputeMinMax(const Vec3q *vec, int size, Vec3f *outMin, Vec3f *outMax);
void ComputeMinMax(const Vec3q *vec, char *mask, int size, Vec3f *outMin, Vec3f *outMax);
void ComputeMinMax(const Vec3q *vec, const floatq *distMask, int size, Vec3f *outMin, Vec3f *outMax);


std::ostream &operator<<(std::ostream&, const Vec3f&);
std::ostream &operator<<(std::ostream&, const floatq&);

struct Plane {
	Plane(Vec3f point, Vec3f normal) :normal(normal) {
		distance = normal | point;
	}
	Plane(Vec3f a, Vec3f b, Vec3f c) {
		normal = ((c - a) ^ (b - a));
		normal *= RSqrt(normal | normal);
		distance = a | normal;
	}
	Plane(Vec3f normal, float distance) :normal(normal), distance(distance) { }
	Plane() { }

	float Intersect(Vec3f origin, Vec3f dir) {
		return -((normal | origin) + distance) / (normal | dir);
	}	
	const Plane operator-() const {
		return Plane(-normal, -distance);
	}

	Vec3f normal;
	float distance;
};

void Intersect(const Plane &a, const Plane &b, Vec3f &dir, Vec3f &point);


std::ostream &operator<<(std::ostream&, const Plane&);

Matrix<Vec4f> Inverse(const Matrix<Vec4f>&);

fwk::Font getFont(const string &name);

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
		fHasMask	= 0x040000,
	};

}

// Intersection
template <class Real,int packetSize,int flags_>
class Isct {
	enum { flags_0 = (flags_ & isct::fShadow? flags_ & 0xff09 : flags_) & 0xffff };

public:
	enum { flags=flags_0 };
	typedef typename TSwitch<i32x4,u32,TIsSame<Real,f32x4>::value>::Result Int;
	typedef typename TSwitch<Vec3q,Vec3f,TIsSame<Real,f32x4>::value>::Result Vec3;
	typedef typename TSwitch<Vec2q,Vec2f,TIsSame<Real,f32x4>::value>::Result Vec2;

private:
	Real    distance[flags&isct::fDistance?packetSize:0];
	Int		object  [flags&isct::fObject  ?packetSize:0];
	Int		element [flags&isct::fElement ?packetSize:0];

	TreeStats stats;

public:
	int lastShadowTri;
	//Dont worry about those if's, they will be optimized out
	
	Isct() { }

	Real &Distance(int q=0) {
		if(!(flags&isct::fDistance)) FATAL("Structure doesnt contain 'distance' member.");
		return distance[q];
	}
	const Real &Distance(int q=0) const {
		if(!(flags&isct::fDistance)) FATAL("Structure doesnt contain 'distance' member.");
		return distance[q];
	}
	Int &Object(int q=0) {
		if(!(flags&isct::fObject)) FATAL("Structure doesnt contain 'object' member.");
		return object[q];
	}
	const Int &Object(int q=0) const {
		if(!(flags&isct::fObject)) FATAL("Structure doesnt contain 'object' member.");
		return object[q];
	}
	Int &Element(int q=0) {
		if(!(flags&isct::fElement)) FATAL("Structure doesnt contain 'element' member.");
		return element[q];
	}
	const Int &Element(int q=0) const {
		if(!(flags&isct::fElement)) FATAL("Structure doesnt contain 'element' member.");
		return element[q];
	}

	int LastShadowTri() const { return lastShadowTri; }
	int &LastShadowTri() { return lastShadowTri; } 

	TreeStats &Stats() { return stats; }
	const TreeStats &Stats() const { return stats; }

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

