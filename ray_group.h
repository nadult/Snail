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
	INLINE f32x4b SSEMask(int idx) const			{ return GetSSEMaskFromBits(bits[idx]); }

	INLINE RaySelector<size/4> &SubSelector(int idx) { return sub[idx]; }
	INLINE const RaySelector<size/4> &SubSelector(int idx) const { return sub[idx]; }

	INLINE char &operator[](int idx)				{ return bits[idx]; }
	INLINE const char &operator[](int idx) const	{ return bits[idx]; }

	INLINE int &Mask4(int idx) 						{ return bits4[idx]; }
	INLINE const int &Mask4(int idx) const 			{ return bits4[idx]; }

	INLINE bool Any() const {
		bool ret=0;
		for(int n=0;n<size/4;n++) ret|=bits4[n]?1:0;
		return ret;
	}

	INLINE void Clear()			{ for(int n=0;n<size/4;n++) bits4[n]=0; }
	INLINE void SelectAll() 	{ for(int n=0;n<size/4;n++) bits4[n]=0x0f0f0f0f; }
};

template <>
class RaySelector<1> {
public:
	enum { size=1, full=0 };
private:
	char bits;
public:
	INLINE f32x4b SSEMask(int idx) const			{ return GetSSEMaskFromBits(bits); }

	INLINE char &operator[](int idx)				{ return bits; }
	INLINE const char &operator[](int idx) const	{ return bits; }

	INLINE char &Mask4(int idx) 					{ return bits; }
	INLINE const char &Mask4(int idx) const 		{ return bits; }
	INLINE bool Any() const { return bits?1:0; }

	INLINE void Clear()			{ bits=0; }
	INLINE void SelectAll() 	{ bits=15; }
};


template <int size_>
class FullSelector {
public:
	enum { size=size_, full=1 };
	static_assert((size&3)==0,"Selector size must be a power of 4");
	
private:
	FullSelector<size/4> sub[4];

public:
	INLINE f32x4b SSEMask(int idx) const			{ return GetSSEMaskFromBits(15); }

	INLINE FullSelector<size/4> &SubSelector(int idx) { return sub[idx]; }
	INLINE const FullSelector<size/4> &SubSelector(int idx) const { return sub[idx]; }

	INLINE char operator[](int idx) const			{ return 15; }
	INLINE int Mask4(int idx) const 				{ return 0xf0f0f0f; }

	INLINE void SelectAll() 	{ }
	INLINE bool Any() const { return 1; }

	INLINE operator RaySelector<size>() const {
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
	INLINE char operator[](int idx) const	{ return 15; }
	INLINE int Mask4(int idx) const 		{ return 15; }

	INLINE void SelectAll() 	{ }
	INLINE bool Any() const { return 1; }

	INLINE operator RaySelector<1>() const {
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

	INLINE RayGroup(const Vec3q *o,const Vec3q *d,const Vec3q *i) :origin(o),dir(d),iDir(i) { }

	template <int tSize>
	INLINE RayGroup(const RayGroup<tSize,sharedOrigin> &rhs,int offset) {
		origin=rhs.OriginPtr()+(sharedOrigin?0:offset);
		dir=rhs.DirPtr()+offset;
		iDir=rhs.IDirPtr()+offset;
	}

	INLINE const Vec3q &Dir(int n) const	{ return dir[n]; }
	INLINE const Vec3q &Origin(int n) const { return origin[sharedOrigin?0:n]; }
	INLINE const Vec3q &IDir(int n) const	{ return iDir[n]; }

	INLINE const Vec3q *DirPtr() const		{ return dir; }
	INLINE const Vec3q *OriginPtr() const	{ return origin; }
	INLINE const Vec3q *IDirPtr() const		{ return iDir; }

private:
	const Vec3q *dir,*iDir,*origin;
};

struct ShadowCache {
	INLINE ShadowCache() { Clear(); }
	INLINE void Clear() { lastTri=~0; }
	INLINE int Size() const { return lastTri==~0?0:1; }
	INLINE void Insert(int idx) { lastTri=idx; }
	INLINE int operator[](int n) const { return lastTri; }

private:
	int lastTri;
};

template <int size_,int flags_>
struct Context {
	enum { size=size_, flags=flags_, sharedOrigin=flags&isct::fShOrig };

	template <int tSize>
	INLINE Context(const Context<tSize,flags> &c,int off) :rays(c.rays,off) {
		distance=c.distance+off;
		object=c.object+off;
		element=c.element+off;
		stats=c.stats;
	}

	INLINE Context(const RayGroup<size,sharedOrigin> &tRays,floatq *dist,i32x4 *obj,
			i32x4 *elem,TreeStats<1> *st=0) :rays(tRays) {
		distance=dist; object=obj; element=elem;
		stats=st;
	}

	INLINE Context(const Vec3q *torig,const Vec3q *tdir,const Vec3q *tidir,floatq *dist,i32x4 *obj,i32x4 *elem,
			TreeStats<1> *st=0) :rays(torig,tdir,tidir) {
		distance=dist; object=obj; element=elem;
		stats=st;
	}

	INLINE const Vec3q &Dir(int q) const { return rays.Dir(q); }
	INLINE const Vec3q &IDir(int q) const { return rays.IDir(q); }
	INLINE const Vec3q &Origin(int q) const { return rays.Origin(q); }

	INLINE f32x4 &Distance(int q) { return distance[q]; }
	INLINE i32x4 &Object(int q) { return object[q]; }
	INLINE i32x4 &Element(int q) { return element[q]; }

	Context<size/4,flags> Split(int part) const {
		const int offset = part * (size / 4);
		Context<size/4, flags> out(RayGroup<size/4,sharedOrigin>(rays,offset),distance+offset,
					 object+offset,element+offset,stats);
		out.mailbox = mailbox;
		return out;
	}

	void UpdateStats(const TreeStats<1> &st) { if(stats) *stats+=st; }

	RayGroup<size,sharedOrigin> rays;
	floatq * __restrict__ __attribute__((aligned(16))) distance;
	i32x4  * __restrict__ __attribute__((aligned(16))) object;
	i32x4  * __restrict__ __attribute__((aligned(16))) element;
	ShadowCache shadowCache;
	ObjectIdxBuffer<4> mailbox;
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

	INLINE const Vec3f &Dir(int) const { return dir; }
	INLINE const Vec3f &IDir(int) const { return iDir; }
	INLINE const Vec3f &Origin(int) const { return origin; }

	INLINE float &Distance(int) { return distance[0]; }
	INLINE int   &Object  (int) { return object  [0]; }
	INLINE int   &Element (int) { return element [0]; }
	
	void UpdateStats(const TreeStats<1> &st) { if(stats) *stats+=st; }

	Vec3f origin,dir,iDir;
	float *__restrict__ distance;
	int   * __restrict__ object;
	int   * __restrict__ element;
	ShadowCache shadowCache;
	TreeStats<1> *stats;
};

#endif

