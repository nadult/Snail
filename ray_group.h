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

template <int size_,bool shared_>
class RayOrigin {
public:
	enum { size=size_, shared=shared_ };
	INLINE RayOrigin() { }
	RayOrigin(const Vec3q *orig) { for(int q=0;q<size;q++) data[q]=orig[q]; }
	RayOrigin(const Vec3q &orig) { for(int q=0;q<size;q++) data[q]=orig; }
	RayOrigin(const Vec3p &orig) {
		Vec3q t; Broadcast(orig,t);
		for(int q=0;q<size;q++) data[q]=t;
	}
	RayOrigin(const RayOrigin<size,1> &rhs) { for(int q=0;q<size;q++) data[q]=rhs[q]; }

	const RayOrigin<size,0> &operator*=(const Matrix<Vec4f> &trans) {
		for(int q=0;q<size;q++) data[q]=trans*data[q];
		return *this;
	}

	INLINE const Vec3q &operator[](int q) const { return data[q]; }
	INLINE void Set(int q,const Vec3q &orig) { data[q]=orig; }
	INLINE void Set(int q,const Vec3p &orig) { Broadcast(orig,data[q]); }

private:
	Vec3q data[size];
};

template <int size_>
class RayOrigin<size_,1> {
public:
	enum { size=size_, shared=1 };

	INLINE RayOrigin() { }
	INLINE RayOrigin(const Vec3q *orig) :data(orig[0].x[0],orig[0].y[0],orig[0].z[0]) { }
	INLINE RayOrigin(const Vec3p &orig) :data(orig) { } 
	INLINE RayOrigin(const Vec3q &orig) :data(orig.x[0],orig.y[0],orig.z[0]) { }

	const RayOrigin<size,1> &operator*=(const Matrix<Vec4f> &trans) {
		data=trans*Vec3f(data);
		return *this;
	}

	INLINE Vec3q operator[](int q) const { Vec3q out; Broadcast(data,out); return out; }
	INLINE void Set(int q,const Vec3q &orig) { data=Vec3p(orig.x[0],orig.y[0],orig.z[0]); }
	INLINE void Set(int q,const Vec3p &orig) { data=orig; }

private:
	Vec3p data;
};

template <int size_,bool inverses_>
class RayDir {
public:
	enum { size=size_, inverses=inverses_ };
	INLINE RayDir() { }
	RayDir(const Vec3q *d) { for(int q=0;q<size;q++) dir[q]=d[q]; }
	RayDir(const Vec3q *d,const Vec3q*) { for(int q=0;q<size;q++) dir[q]=d[q]; }
	RayDir(const RayDir<size,1> &rhs) { for(int q=0;q<size;q++) dir[q]=rhs[q]; }

	INLINE const Vec3q &operator[](int q) const { return dir[q]; }
	INLINE Vec3q Inv(int q) const { return VInv(dir[q]); }
	INLINE void Set(int q,const Vec3q &d) { dir[q]=d; }
	INLINE void Set(int q,const Vec3q &d,const Vec3q&) { dir[q]=d; }

private:
	Vec3q dir[size];
};

template <int size_>
class RayDir<size_,1> {
public:
	enum { size=size_, inverses=1 };
	INLINE RayDir() { }
	RayDir(const Vec3q *d) { for(int q=0;q<size;q++) { dir[q]=d[q]; inv[q]=VInv(dir[q]); } } 
	RayDir(const Vec3q *d,const Vec3q *i) {
		for(int q=0;q<size;q++) dir[q]=d[q];
		for(int q=0;q<size;q++) inv[q]=i[q];
	}
	RayDir(const RayDir<size,0> &rhs) { for(int q=0;q<size;q++) { dir[q]=rhs[q]; inv[q]=VInv(dir[q]); } }

	INLINE const Vec3q &operator[](int q) const { return dir[q]; }
	INLINE const Vec3q &Inv(int q) const { return inv[q]; }
	INLINE void Set(int q,const Vec3q &d) { dir[q]=d; inv[q]=VInv(dir[q]); }
	INLINE void Set(int q,const Vec3q &d,const Vec3q &i) { dir[q]=d; inv[q]=i; }

private:
	Vec3q dir[size];
	Vec3q inv[size];
};

template <int size_,bool enabled_>
class RayMaxDist {
public:
	enum { size=size_, enabled=enabled_ };
	INLINE RayMaxDist() { }
	INLINE RayMaxDist(const floatq&) { }
	INLINE RayMaxDist(const floatq*) { }

	INLINE floatq operator[](int q) const { return floatq(1.0f/0.0f); }
	INLINE void Set(int q,floatq dist) { }
};

template <int size_>
class RayMaxDist<size_,1> {
public:
	enum { size=size_, enabled=1 };
	INLINE RayMaxDist() { }
	INLINE RayMaxDist(const floatq &dist) { for(int q=0;q<size;q++) data[q]=dist; }
	INLINE RayMaxDist(const floatq *dist) { for(int q=0;q<size;q++) data[q]=dist[q]; }
	INLINE RayMaxDist(const RayMaxDist<size,0> &rhs) { for(int q=0;q<size;q++) data[q]=rhs[q]; }

	INLINE const floatq &operator[](int q) const { return data[q]; }
	INLINE void Set(int q,floatq dist) { data[q]=dist; }

private:
	floatq data[size];
};


template <int size_,int flags_>
class RayGroup {
public:
	enum { size=size_, flags=flags_, sharedOrigin=flags&isct::fShOrig?1:0,
			inverses=flags&isct::fInvDir?1:0, hasMaxDist=flags&isct::fMaxDist?1:0 };

	RayGroup() :lastShadowTri(-1) { }
	template <int fl>
	RayGroup(const RayGroup<size,fl> &rhs) {
		origin=rhs.origin;
		dir=rhs.dir;
		maxDist=rhs.maxDist;
		lastShadowTri=rhs.lastShadowTri;
	}
	RayGroup(const RayOrigin<size,sharedOrigin> &o,const RayDir<size,inverses> &d,
			  const RayMaxDist<size,hasMaxDist> &dst) :origin(o),dir(d),maxDist(dst),lastShadowTri(-1) { }
	RayGroup(const RayOrigin<size,sharedOrigin> &o,const RayDir<size,inverses> &d) :origin(o),dir(d),lastShadowTri(-1) { }

	template <int fl>
	void Split(RayGroup<size/4,fl> groups[4]) const {
		for(int k=0;k<4;k++) for(int q=0;q<size/4;q++) groups[k].origin.Set(q,Origin(k*(size/4)+q));
		if(fl&isct::fInvDir)
			for(int k=0;k<4;k++) for(int q=0;q<size/4;q++) groups[k].dir.Set(q,Dir(k*(size/4)+q),IDir(k*(size/4)+q));
		else
			for(int k=0;k<4;k++) for(int q=0;q<size/4;q++) groups[k].dir.Set(q,Dir(k*(size/4)+q));
		for(int k=0;k<4;k++) for(int q=0;q<size/4;q++) groups[k].maxDist.Set(q,MaxDist(k*(size/4)+q));
	}

	void operator*=(const Matrix<Vec4f> &trans) {
		origin*=trans;
		for(int q=0;q<size;q++) dir.Set(q,trans&dir[q]);
		//TODO: scale max dist properly
	}

	INLINE Vec3q Origin(int q) const { return origin[q]; }
	INLINE Vec3q Dir(int q) const { return dir[q]; }
	INLINE Vec3q IDir(int q) const { return dir.Inv(q); }
	INLINE floatq MaxDist(int q) const { return maxDist[q]; }

//private:
	RayOrigin<size,sharedOrigin> origin;
	RayDir<size,inverses> dir;
	RayMaxDist<size,hasMaxDist> maxDist;
	int lastShadowTri;
};

#endif

