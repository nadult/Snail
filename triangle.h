#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "rtbase.h"


template <class Triangle>
class FastEdgeNormals
{
public:
	inline const Vec3p &Edge1Normal(const Triangle *base) const { return e1n; }
	inline const Vec3p &Edge2Normal(const Triangle *base) const { return e2n; }
	inline const Vec3p &Edge3Normal(const Triangle *base) const { return e3n; }

protected:
	void ComputeEdgeNormals(Triangle *base) {
		Vec3p nrm=base->Nrm();
		e1n=nrm^base->Edge1();
		e2n=nrm^base->Edge2();
		e3n=nrm^base->Edge3();
		e1n*=RSqrt(e1n|e1n);
		e2n*=RSqrt(e2n|e2n);
		e3n*=RSqrt(e3n|e3n);
	}

	Vec3p e1n,e2n,e3n;
};

template <class Triangle>
class SlowEdgeNormals
{
public:
	inline Vec3p Edge1Normal(const Triangle *base) const { Vec3p tmp=base->Nrm()^base->Edge1(); return tmp*RSqrt(tmp|tmp); }
	inline Vec3p Edge2Normal(const Triangle *base) const { Vec3p tmp=base->Nrm()^base->Edge2(); return tmp*RSqrt(tmp|tmp); }
	inline Vec3p Edge3Normal(const Triangle *base) const { Vec3p tmp=base->Nrm()^base->Edge3(); return tmp*RSqrt(tmp|tmp); }

protected:
	void ComputeEdgeNormals(Triangle *base) { }
};


template <template <class> class TEdgeNormals>
class TTriangle: public TEdgeNormals < TTriangle <TEdgeNormals> >
{
public:
	typedef TEdgeNormals< TTriangle<TEdgeNormals> > EdgeNormals;

	TTriangle(const Vec3f &ta,const Vec3f &tb,const Vec3f &tc) {
		Convert(ta,a); Convert(tb,b); Convert(tc,c);
		ComputeData();
	}
	TTriangle() {
	}

	inline Vec3p Edge1() const { return b-a; }
	inline Vec3p Edge2() const { return c-b; }
	inline Vec3p Edge3() const { return a-c; }

	inline Vec3p Edge1Normal() const { return EdgeNormals::Edge1Normal(this); }
	inline Vec3p Edge2Normal() const { return EdgeNormals::Edge2Normal(this); }
	inline Vec3p Edge3Normal() const { return EdgeNormals::Edge3Normal(this); }

	inline Vec3p Nrm() const { return Vec3p(plane); }

	inline Vec3p BoundMin() const { return VMin(a,VMin(b,c)); }
	inline Vec3p BoundMax() const { return VMax(a,VMax(b,c)); }

	template <class Vec>
	inline Vec Normal(const Vec&) const { return Vec(Nrm()); }

	template <class VecO,class Vec>
	typename Vec::TScalar Collide(const VecO &rOrig,const Vec &rDir) const NOINLINE;

	int BeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL,float epsC) const NOINLINE;

	void SetFlag1(uint value) { ((uint*)&a)[3]=value; }
	void SetFlag2(uint value) { ((uint*)&b)[3]=value; }
	void SetFlag3(uint value) { ((uint*)&c)[3]=value; }
	uint GetFlag1() const { return ((uint*)&a)[3]; }
	uint GetFlag2() const { return ((uint*)&b)[3]; }
	uint GetFlag3() const { return ((uint*)&c)[3]; }

private:
	void ComputeData() {
		Vec3p nrm=(b-a)^(c-a);
		nrm*=RSqrt(nrm|nrm);
		plane=Vec4p(nrm.x,nrm.y,nrm.z,nrm|a);
		e1ce2=(b-a)^(c-a);
		EdgeNormals::ComputeEdgeNormals(this);
	}

public:
	Vec3p a,b,c;
	Vec4p plane;
	Vec3p e1ce2;
};

template <template <class> class EN> template <class VecO,class Vec>
typename Vec::TScalar TTriangle<EN>::Collide(const VecO &rOrig,const Vec &rDir) const {
	typedef typename Vec::TScalar base;
	typedef typename Vec::TBool Bool;

	base out=Const<base,-1>();

	base det = rDir|e1ce2;
	VecO tvec = rOrig-VecO(a);
	base u = rDir|(VecO(b-a)^tvec);
	base v = rDir|(tvec^VecO(c-a));
	Bool test=Min(u,v)>=Const<base,0>()&&u+v<=det;

	if (ForAny(test)) {
		base dist=-(tvec|Nrm())*Inv(rDir|Nrm());
		out=Condition(test,dist,out);
	}

	return out;
}

template <template <class> class EN>
int TTriangle<EN>::BeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL,float epsC) const {
	float dot=Abs(dir|Nrm());

	if(ForAny(dot<=Const<float,1,100000>()))
		return 1;

	float idot=Inv(dir|Nrm());

	float t=-((orig-a)|Nrm())*idot;
	Vec3p col=orig+dir*t;

	if(ForAny(t<-epsC*idot)) return 0;

	float epsilon=(epsL*t+epsC)*idot;

	float dist[3]={
		(col-a)|Edge1Normal(),
		(col-b)|Edge2Normal(),
		(col-c)|Edge3Normal() };
	float min=Min(dist[0],Min(dist[1],dist[2]));

	if(min<-epsilon) return 0;
	return (min>epsilon?2:1);
}



typedef TTriangle<FastEdgeNormals> Triangle;


template <class Triangle>
class IndexedTriangle {
public:

	template <class Array>
	IndexedTriangle(uint ind,const Array &arr) :ref(arr[ind]),index(ind) {
	}

public:
	uint index;
	Triangle &ref;
};

#endif

