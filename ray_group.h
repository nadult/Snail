#ifndef RTRACER_RAY_GROUP_H
#define RTRACER_RAY_GROUP_H

#include "rtbase.h"

extern f32x4b GetSSEMaskFromBits_array[16];

INLINE f32x4b GetSSEMaskFromBits(u8 bits) {
	assert(bits<=15);
	/*
	union { u32 tmp[4]; __m128 out; };

	tmp[0]=bits&1?~0:0;
	tmp[1]=bits&2?~0:0;
	tmp[2]=bits&4?~0:0;
	tmp[3]=bits&8?~0:0;

	return out; */
	return GetSSEMaskFromBits_array[bits];
}

INLINE int CountMaskBits(u8 bits) {
	assert(bits <= 15);
	static const char count[16] = { 0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4 };
	return count[bits];
}

typedef int RayIndex;

// this class stores a list of active rays
// with their masks
template <int size_>
class RaySelector {
public:
	enum { size=size_, full=0 };
	static_assert((size&3)==0,"Selector size must be a power of 4");

private:
	union {
		RaySelector<size/4> sub[4];
		char bits[size];
		int bits4[size/4];
	};
public:
	f32x4b SSEMask(int idx) const			{ return GetSSEMaskFromBits(bits[idx]); }

	RaySelector<size/4> &SubSelector(int idx) { return sub[idx]; }
	const RaySelector<size/4> &SubSelector(int idx) const { return sub[idx]; }

	char &operator[](int idx)				{ return bits[idx]; }
	const char &operator[](int idx) const	{ return bits[idx]; }

	int &Mask4(int idx) 						{ return bits4[idx]; }
	const int &Mask4(int idx) const 			{ return bits4[idx]; }

	bool Any() const {
		bool ret=0;
		for(int n=0;n<size/4;n++) ret|=bits4[n]?1:0;
		return ret;
	}
	bool All() const {
		bool ret = 1;
		for(int n = 0; n < size / 4; n++)
			ret &= bits4[n] == 15;
		return ret;
	}

	void Clear()			{ for(int n=0;n<size/4;n++) bits4[n]=0; }
	void SelectAll() 	{ for(int n=0;n<size/4;n++) bits4[n]=0x0f0f0f0f; }
};

template <>
class RaySelector<1> {
public:
	enum { size=1, full=0 };
private:
	char bits;
public:
	f32x4b SSEMask(int idx) const			{ return GetSSEMaskFromBits(bits); }

	char &operator[](int idx)				{ return bits; }
	const char &operator[](int idx) const	{ return bits; }

	char &Mask4(int idx) 					{ return bits; }
	const char &Mask4(int idx) const 		{ return bits; }
	bool Any() const { return bits? 1 : 0; }
	bool All() const { return bits == 15; }

	void Clear()		{ bits = 0; }
	void SelectAll() 	{ bits = 15; }
};


template <int size_>
class FullSelector {
public:
	enum { size=size_, full=1 };
	static_assert((size&3)==0,"Selector size must be a power of 4");
	
private:
	FullSelector<size/4> sub[4];

public:
	f32x4b SSEMask(int idx) const			{ return GetSSEMaskFromBits(15); }

	FullSelector<size/4> &SubSelector(int idx) { return sub[idx]; }
	const FullSelector<size/4> &SubSelector(int idx) const { return sub[idx]; }

	//TODO ...
	char &operator[](int idx) { static char t = 15; return t; }

	char operator[](int idx) const			{ return 15; }
	int Mask4(int idx) const 				{ return 0xf0f0f0f; }

	void SelectAll() 	{ }
	bool Any() const { return 1; }
	bool All() const { return 1; }

	operator RaySelector<size>() const {
		RaySelector<size> sel;
		sel.SelectAll();
		return sel;
	}
};

template <>
class FullSelector<1> {
public:
	enum { size=1, full=1 };
	
public:
	char operator[](int idx) const	{ return 15; }
	int Mask4(int idx) const 		{ return 15; }

	void SelectAll() 	{ }
	bool Any() const { return 1; }

	operator RaySelector<1>() const {
		RaySelector<1> sel;
		sel.SelectAll();
		return sel;
	}
};

// returns 8 for mixed rays
inline int GetVecSign(const Vec3q &dir) {
	int x=SignMask(dir.x),y=SignMask(dir.y),z=SignMask(dir.z);
	int out=(x?1:0)+(y?2:0)+(z?4:0);

	if((x>0&&x<15)||(y>0&&y<15)||(z>0&&z<15)) out=8;
	else if(ForAny(	Min(Abs(dir.x),Min(Abs(dir.y),Abs(dir.z))) < f32x4(0.000001f))) out=8;

	return out;
}

template <class Selector1,class Selector2>
inline void SplitSelectorsBySign(const Selector1 &in,Selector2 out[9],Vec3q *dir) {
	for(int n=0;n<9;n++) out[n].Clear();
	for(int i=0;i<in.Num();i++) {
		int n=in[i];
		out[GetVecSign(dir[n])].Add(in,i);
	}
}


template <int size_,bool sharedOrigin_>
class RayGroup
{
public:
	enum { size=size_, sharedOrigin=sharedOrigin_ };

	RayGroup(const Vec3q *o, const Vec3q *d, const Vec3q *i) :origin(o), dir(d), iDir(i) { }

	template <int tSize>
	RayGroup(const RayGroup<tSize,sharedOrigin> &rhs,int offset) {
		origin=rhs.OriginPtr()+(sharedOrigin?0:offset);
		dir=rhs.DirPtr()+offset;
		iDir=rhs.IDirPtr()+offset;
	}

	const Vec3q &Dir(int n) const	{ return dir[n]; }
	const Vec3q &Origin(int n) const { return origin[sharedOrigin?0:n]; }
	const Vec3q &IDir(int n) const	{ return iDir[n]; }

	const Vec3q *DirPtr() const		{ return dir; }
	const Vec3q *OriginPtr() const	{ return origin; }
	const Vec3q *IDirPtr() const		{ return iDir; }

private:
	const Vec3q *dir,*iDir,*origin;
};

//TODO specjalizacja dla roznych originow?
class CornerRays
{
public:
	CornerRays() { }
	template <int size, bool shared>
	explicit CornerRays(const RayGroup<size, shared> &gr) {
		Assert(shared);

		Vec3f td[4];
		td[0] = ExtractN(gr. Dir( size == 4? 0 : size == 16?  0 : size == 32?  0 : size == 64?  0 : size == 128? 0  : 0  ), 0);
		td[1] = ExtractN(gr. Dir( size == 4? 1 : size == 16?  3 : size == 32? 19 : size == 64? 21 : size == 128? 85 : 85 ), 1);
		td[2] = ExtractN(gr. Dir( size == 4? 2 : size == 16? 10 : size == 32? 10 : size == 64? 42 : size == 128? 42 : 170), 2);
		td[3] = ExtractN(gr. Dir( size == 4? 3 : size == 16? 15 : size == 32? 31 : size == 64? 63 : size == 128? 127: 255), 3);
		Convert(td, dir);
		td[0] = ExtractN(gr.IDir( size == 4? 0 : size == 16?  0 : size == 32?  0 : size == 64?  0 : size == 128? 0  : 0  ), 0);
		td[1] = ExtractN(gr.IDir( size == 4? 1 : size == 16?  3 : size == 32? 19 : size == 64? 21 : size == 128? 85 : 85 ), 1);
		td[2] = ExtractN(gr.IDir( size == 4? 2 : size == 16? 10 : size == 32? 10 : size == 64? 42 : size == 128? 42 : 170), 2);
		td[3] = ExtractN(gr.IDir( size == 4? 3 : size == 16? 15 : size == 32? 31 : size == 64? 63 : size == 128? 127: 255), 3);
		Convert(td, idir);
		origin = ExtractN(gr.Origin(0), 0);
	}

	// Order:
	// 0  -  1
	// |     |
	// 2  -  3
	Vec3q dir, idir;
	Vec3f origin;
};

class RayInterval {
public:
	RayInterval() { }
	template <int size, bool shared>
	explicit RayInterval(const RayGroup<size, shared> &rays) {
		Assert(shared);

		ComputeMinMax<size>(&rays.Dir(0), &minDir, &maxDir);
		ComputeMinMax<size>(&rays.IDir(0), &minIDir, &maxIDir);
		origin = ExtractN(rays.Origin(0), 0);
		
		ix = floatq(minIDir.x, maxIDir.x, minIDir.x, maxIDir.x);
		iy = floatq(minIDir.y, maxIDir.y, minIDir.y, maxIDir.y);
		iz = floatq(minIDir.z, maxIDir.z, minIDir.z, maxIDir.z);
	}
	template <int size, bool shared, class Selector>
	RayInterval(const RayGroup<size, shared> &rays, const Selector &selector) {
		Assert(shared);

		ComputeMinMax<size>(&rays.Dir(0), &minDir, &maxDir, selector);
		ComputeMinMax<size>(&rays.IDir(0), &minIDir, &maxIDir, selector);
		origin = ExtractN(rays.Origin(0), 0);
		
		ix = floatq(minIDir.x, maxIDir.x, minIDir.x, maxIDir.x);
		iy = floatq(minIDir.y, maxIDir.y, minIDir.y, maxIDir.y);
		iz = floatq(minIDir.z, maxIDir.z, minIDir.z, maxIDir.z);
	}
	//TODO: zle; sa dziury np. na sponzie
/*	RayInterval(const CornerRays &rays) {
		minIDir = Minimize(rays.idir);
		minDir = Minimize(rays.dir);
		maxIDir = Maximize(rays.idir);
		maxDir = Maximize(rays.dir);
		origin = rays.origin;

		ix = floatq(minIDir.x, maxIDir.x, minIDir.x, maxIDir.x);
		iy = floatq(minIDir.y, maxIDir.y, minIDir.y, maxIDir.y);
		iz = floatq(minIDir.z, maxIDir.z, minIDir.z, maxIDir.z);
	} */

	floatq ix, iy, iz;
	Vec3f minIDir, maxIDir, minDir, maxDir;
	Vec3f origin;
};

struct ShadowCache {
	ShadowCache() { Clear(); }
	void Clear() { lastTri=~0; }
	int Size() const { return lastTri==~0?0:1; }
	void Insert(int idx) { lastTri=idx; }
	int operator[](int n) const { return lastTri; }

private:
	int lastTri;
};

template <int size_,int flags_>
struct Context {
	enum { size=size_, flags=flags_, sharedOrigin=flags&isct::fShOrig };

	template <int tSize>
	Context(const Context<tSize,flags> &c,int off) :rays(c.rays,off) {
		distance=c.distance+off;
		object=c.object+off;
		element=c.element+off;
		stats=c.stats;
	}

	Context(const RayGroup<size,sharedOrigin> &tRays,floatq *dist,i32x4 *obj,
			i32x4 *elem,TreeStats<1> *st=0) :rays(tRays) {
		distance=dist; object=obj; element=elem;
		stats=st;
	}

	Context(const Vec3q *torig,const Vec3q *tdir,const Vec3q *tidir,floatq *dist,i32x4 *obj,i32x4 *elem,
			TreeStats<1> *st=0) :rays(torig,tdir,tidir) {
		distance=dist; object=obj; element=elem;
		stats=st;
	}

	const Vec3q &Dir(int q) const { return rays.Dir(q); }
	const Vec3q &IDir(int q) const { return rays.IDir(q); }
	const Vec3q &Origin(int q) const { return rays.Origin(q); }

	f32x4 &Distance(int q) { return distance[q]; }
	i32x4 &Object(int q) { return object[q]; }
	i32x4 &Element(int q) { return element[q]; }

	const Context<size/4,flags> Split(int part) const {
		const int offset = part * (size / 4);
		Context<size/4, flags> out(RayGroup<size/4,sharedOrigin>(rays, offset), distance + offset,
					 object+offset,element+offset,stats);
		return out;
	}

	const Context<size/2,flags> Split2(int part) const {
		const int offset = part * (size / 2);
		Context<size/2, flags> out(RayGroup<size/2, sharedOrigin>(rays, offset), distance + offset,
					 object+offset, element + offset, stats);
		return out;
	}

	void UpdateStats(const TreeStats<1> &st) { if(stats) *stats+=st; }

	RayGroup<size,sharedOrigin> rays;
	floatq * __restrict__ __attribute__((aligned(16))) distance;
	i32x4  * __restrict__ __attribute__((aligned(16))) object;
	i32x4  * __restrict__ __attribute__((aligned(16))) element;
	ShadowCache shadowCache;
	TreeStats<1> *stats;
};

template <int flags_>
struct FContext {
	enum { flags=flags_, sharedOrigin=flags&isct::fShOrig };

	FContext(const Context<1,flags> &c,int o) {
		const Vec3q &tOrig=c.Origin(0);
		const Vec3q &tDir=c.Dir(0);
		const Vec3q &tIDir=c.IDir(0);
		origin=Vec3f(tOrig.x[o],tOrig.y[o],tOrig.z[o]);
		dir=Vec3f(tDir.x[o],tDir.y[o],tDir.z[o]);
		iDir=Vec3f(tIDir.x[o],tIDir.y[o],tIDir.z[o]);
		
		distance = &c.distance[0][o];
		object   = &c.object  [0][o];
		element  = &c.element [0][o];
		if(flags&isct::fShadow) shadowCache=c.shadowCache;
		stats=c.stats;
	}

	const Vec3f &Dir(int) const { return dir; }
	const Vec3f &IDir(int) const { return iDir; }
	const Vec3f &Origin(int) const { return origin; }

	float &Distance(int) { return distance[0]; }
	int   &Object  (int) { return object  [0]; }
	int   &Element (int) { return element [0]; }
	
	void UpdateStats(const TreeStats<1> &st) { if(stats) *stats+=st; }

	Vec3f origin,dir,iDir;
	float *__restrict__ distance;
	int   * __restrict__ object;
	int   * __restrict__ element;
	ShadowCache shadowCache;
	TreeStats<1> *stats;
};

#endif

