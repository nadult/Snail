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
	assert(bits<=15);
	char count[16]= { 0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4 };
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

	INLINE void Clear()			{ for(int n=0;n<size;n++) bits[n]=0; }
	INLINE void SelectAll() 	{ for(int n=0;n<size;n++) bits[n]=15; }
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
	else if(ForAny(	Min(Abs(dir.x),Min(Abs(dir.y),Abs(dir.z)))<ConstEpsilon<f32x4>() )) out=8;

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
	

template <int size_,bool tSharedOrigin=0,bool tPrecomputedInverses=0>
class RayGroup
{
public:
	enum { sharedOrigin=tSharedOrigin, precomputedInverses=tPrecomputedInverses, size=size_ };

	template <class Selector1,class Selector2>
	void GenSelectors(const Selector1 &oldSelector,Selector2 sel[9]) {
		SplitSelectorsBySign(oldSelector,sel,&Dir(0));
	}

	INLINE RayGroup(Vec3q *o,Vec3q *d,Vec3q *i=0) :dir(d),idir(i),origin(o) { assert(!precomputedInverses||idir); }

	template <int tSize,bool tPrecomputed>
	INLINE RayGroup(const RayGroup<tSize,sharedOrigin,tPrecomputed> &other,int shift)
		:dir(other.dir+shift),idir(precomputedInverses&&tPrecomputed?other.idir+shift:0)
		,origin(sharedOrigin?other.origin:other.origin+shift) { }

	INLINE Vec3q &Dir(int n)				{ return dir[n]; }
	INLINE const Vec3q &Dir(int n) const	{ return dir[n]; }

	INLINE Vec3q &Origin(int n)				{ return origin[sharedOrigin?0:n]; }
	INLINE const Vec3q &Origin(int n) const { return origin[sharedOrigin?0:n]; }

	INLINE Vec3q IDir(int n) const			{ return precomputedInverses?idir[n]:VInv(dir[n]); }

	INLINE Vec3q *DirPtr()		{ return dir; }
	INLINE Vec3q *OriginPtr()	{ return origin; }
	INLINE Vec3q *IDirPtr()		{ return precomputedInverses?idir:0; }

//private:
	Vec3q *dir,*idir,*origin;
};

#endif

