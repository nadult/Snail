//#include <boost/static_assert.hpp>
#include "veclib.h"

template <int size>
class RaySelector {
	int num;
	unsigned char signMask;
	unsigned char idx[size];
public:
	INLINE RaySelector() :num(0),signMask(8) {
	}

	INLINE int Num() const {
		return num;
	}
	INLINE int Last() const {
		return idx[num-1];
	}
	INLINE int Idx(int n) const {
		return idx[n];
	}
	INLINE int operator[](int n) const {
		return idx[n];
	}
	// Moves last index to n,
	// decreases number of selected rays
	INLINE void Disable(int n) {
		idx[n]=idx[--num];
	}
	INLINE void Add(int n) {
		idx[num++]=n;
	}
	INLINE void Clear() {
		num=0;
	}
	INLINE void SelectAll() {
		num=size; signMask=8;
		for(int n=0;n<size;n++) idx[n]=n;
	}
	INLINE char SignMask() const {
		return signMask;
	}
	INLINE int SignMaskX() const {
		return signMask&1;
	}
	INLINE int SignMaskY() const {
		return (signMask&2)>>1;
	}
	INLINE int SignMaskZ() const {
		return signMask>>2;
	}
	INLINE bool IsCoherent() const {
		return signMask!=8;
	}
	INLINE void SetSignMask(char mask) {
		signMask=mask;
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
	INLINE Vec3q &Dir(int n) {
		return dir[n];
	}
	INLINE const Vec3q &Dir(int n) const {
		return dir[n];
	}
	INLINE Vec3q &Origin(int n) {
		return origin[n];
	}
	INLINE const Vec3q &Origin(int n) const {
		return origin[n];
	}

	INLINE Vec3q *DirPtr() {
		return dir;
	}
	INLINE Vec3q *OriginPtr() {
		return origin;
	}
};

template <int size>
class RayStore<size,true>
{
	Vec3q *origin;
	Vec3q *dir;

public:
	INLINE RayStore(Vec3q *d,Vec3q *o) :dir(d),origin(o) {
	}
	INLINE Vec3q &Dir(int n) {
		return dir[n];
	}
	INLINE const Vec3q &Dir(int n) const {
		return dir[n];
	}
	INLINE Vec3q &Origin(int n) {
		return origin[0];
	}
	INLINE const Vec3q &Origin(int n) const {
		return origin[0];
	}

	INLINE Vec3q *DirPtr() {
		return dir;
	}
	INLINE Vec3q *OriginPtr() {
		return origin;
	}
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
				if(ForAny(	Abs(r.x)<ConstEpsilon<SSEReal>()||
							Abs(r.y)<ConstEpsilon<SSEReal>()||
							Abs(r.z)<ConstEpsilon<SSEReal>()))
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

		for(int n=0;n<9;n++) { sel[n].Clear(); sel[n].SetSignMask(n); }
		for(int i=0;i<oldSelectors.Num();i++) {
			int n=oldSelectors[i];
			sel[raySign[n]].Add(n);
		}
	}
	INLINE int RaySignMask(int n) const {
		return raySign[n];
	}

	INLINE RayGroup(RayStore<size,singleOrigin> &ref) :store(ref),signsCalculated(0) {
	}
	INLINE RayGroup(Vec3q *dir,Vec3q *origin) :store(dir,origin),signsCalculated(0) {
	}

	INLINE Vec3q &Dir(int n) {
		return store.Dir(n);
	}
	INLINE const Vec3q &Dir(int n) const {
		return store.Dir(n);
	}

	INLINE Vec3q &Origin(int n) {
		return store.Origin(n);
	}
	INLINE const Vec3q &Origin(int n) const {
		return store.Origin(n);
	}

	// Iloœæ promieni, w tym tych wy³¹czonych
	INLINE int Size() const { return size; }

};
