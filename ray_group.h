#ifndef RTRACER_RAY_GROUP_H
#define RTRACER_RAY_GROUP_H

#include "rtbase.h"

extern f32x4b GetSSEMaskFromBits_array[16];

// bits must be <= 15
//
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

// Before you start using this class, try to understand how
// it works, whats most important: there are two kinds of
// indexes:
// - RayIndex which never changes, you can use it to address
//   ray data stored in external array
// - ray number (of type int) which is used to iterate over
//   active rays ( from 0 to Num()-1 )
template <int size>
class RaySelector {
	u8 idx[size];
	char bits[size];
	int num;
public:
	INLINE RaySelector() :num(0) { }

	INLINE int Num() const					{ return num; }
	INLINE RayIndex Last() const			{ return idx[num-1]; }
	INLINE RayIndex Idx(int n) const		{ return idx[n]; }
	INLINE RayIndex operator[](int n) const { return idx[n]; }

	// Moves last index to n,
	// decreases number of selected rays
	INLINE void Disable(int n)				{ idx[n]=idx[--num]; }
	INLINE bool DisableWithMask(int n,const f32x4b &m) {
		int i=idx[n];
		int newMask=_mm_movemask_ps(m.m);
		bits[i]&=~newMask;
		bool disable=!bits[i];
		if(disable) idx[n]=idx[--num];
		return disable;
	}
	INLINE bool DisableWithBitMask(int n,int newMask) {
		int i=idx[n];
		bits[i]&=~newMask;
		bool disable=!bits[i];
		if(disable) idx[n]=idx[--num];
		return disable;
	}

	INLINE void Add(RayIndex i,int bitMask=15)	{ idx[num++]=i; bits[i]=bitMask; }
	INLINE int  BitMask(RayIndex i) const		{ return bits[i]; }
	INLINE f32x4b Mask(RayIndex i) const		{ return GetSSEMaskFromBits(bits[i]); }

	INLINE void SetBitMask(RayIndex i,int m) { bits[i]=m; }
	INLINE void SetMask(RayIndex i,const f32x4b &m) { return bits[i]=_mm_movemask_ps(m.m); }

	INLINE void Clear() { num=0; }
	INLINE void SelectAll() {
		num=size;
		for(int n=0;n<size;n++) { idx[n]=n; bits[n]=15; }
	}
};

template <int tRecLevel,bool tSingleOrigin>
class RayGroup;

template <int size,bool singleOrigin>
class RayStore
{
	Vec3q *dir;
	Vec3q *origin;

public:
	INLINE RayStore(Vec3q *d,Vec3q *o) :dir(d),origin(o) {
	}
	INLINE Vec3q &Dir(int n)				{ return dir[n]; }
	INLINE const Vec3q &Dir(int n) const	{ return dir[n]; }
	INLINE Vec3q &Origin(int n)				{ return origin[n]; }
	INLINE const Vec3q &Origin(int n) const	{ return origin[n]; }

	INLINE Vec3q *DirPtr()		{ return dir; }
	INLINE Vec3q *OriginPtr()	{ return origin; }
};

template <int size>
class RayStore<size,true>
{
	Vec3q *origin;
	Vec3q *dir;

public:
	INLINE RayStore(Vec3q *d,Vec3q *o) :dir(d),origin(o) { }

	INLINE Vec3q &Dir(int n)					{ return dir[n]; }
	INLINE const Vec3q &Dir(int n) const		{ return dir[n]; }
	INLINE Vec3q &Origin(int n)					{ return origin[0]; }
	INLINE const Vec3q &Origin(int n) const		{ return origin[0]; }

	INLINE Vec3q *DirPtr()		{ return dir; }
	INLINE Vec3q *OriginPtr()	{ return origin; }
};

/*!
	recLevel:	nQuads		resolution:
	0:			1			2x2
	1:			4			4x4
	2:			16			8x8
	3:			64			16x16

	Functions starting with E return
	data for nth enabled ray
*/
template <int tRecLevel,bool tSingleOrigin>
class RayGroup
{
public:
	enum { recLevel=tRecLevel, size=1<<recLevel*2, singleOrigin=tSingleOrigin };

private:
	RayStore<size,singleOrigin> store;
	bool signsCalculated;
	char raySign[size]; //0 - 7: all rays in group has same dir   8: mixed

public:
	void ComputeSignMasks() {
		if(signsCalculated) return;

		for(int n=0;n<size;n++) {
			Vec3q &r=Dir(n);
			int x=SignMask(r.x),y=SignMask(r.y),z=SignMask(r.z);
			raySign[n]=(x?1:0)+(y?2:0)+(z?4:0);
			if((x>0&&x<15)||(y>0&&y<15)||(z>0&&z<15)) raySign[n]=8;
			else {
				if(ForAny(	Abs(r.x)<ConstEpsilon<f32x4>()||
							Abs(r.y)<ConstEpsilon<f32x4>()||
							Abs(r.z)<ConstEpsilon<f32x4>()))
					raySign[n]=8;
			}
		}
		signsCalculated=1;
	}

	// Last group: mixed rays
	void GenSelectors(RaySelector<size> sel[9]) {
		if(!signsCalculated) ComputeSignMasks();

		for(int n=0;n<9;n++) { sel[n].Clear(); sel[n].SetSignMask(n); }
		for(int n=0;n<size;n++) sel[raySign[n]].Add(n);
	}
	void GenSelectors(const RaySelector<size> oldSelectors,RaySelector<size> sel[9]) {
		if(!signsCalculated) ComputeSignMasks();

		for(int n=0;n<9;n++) sel[n].Clear();
		for(int i=0;i<oldSelectors.Num();i++) {
			int n=oldSelectors[i];
			sel[raySign[n]].Add(n,oldSelectors.BitMask(n));
		}
	}
	INLINE int RaySignMask(int n) const { return raySign[n]; }

	INLINE RayGroup(RayStore<size,singleOrigin> &ref) :store(ref),signsCalculated(0) { }
	INLINE RayGroup(Vec3q *dir,Vec3q *origin) :store(dir,origin),signsCalculated(0) { }

	INLINE Vec3q &Dir(int n)				{ return store.Dir(n); }
	INLINE const Vec3q &Dir(int n) const	{ return store.Dir(n); }

	INLINE Vec3q &Origin(int n)				{ return store.Origin(n); }
	INLINE const Vec3q &Origin(int n) const { return store.Origin(n); }

	// Including disabled rays
	INLINE int Size() const { return size; }

};

#endif

