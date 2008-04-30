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
	template <template <class> class T1>
	TTriangle(const TTriangle<T1> &other) {
		a=other.P1(); b=other.P2(); c=other.P3();
		SetFlag1(other.GetFlag1());
		SetFlag2(other.GetFlag2());
		ComputeData();
	}
	TTriangle() {
	}

	inline Vec3p P1() const { return a; }
	inline Vec3p P2() const { return b; }
	inline Vec3p P3() const { return c; }

	inline Vec3p Edge1() const { return b-a; }
	inline Vec3p Edge2() const { return c-b; }
	inline Vec3p Edge3() const { return a-c; }

	inline Vec3p Edge1Normal() const { return EdgeNormals::Edge1Normal(this); }
	inline Vec3p Edge2Normal() const { return EdgeNormals::Edge2Normal(this); }
	inline Vec3p Edge3Normal() const { return EdgeNormals::Edge3Normal(this); }

	inline Vec3p Nrm() const { return Vec3p(plane); }

	inline float InvSize() const { return ((float*)&c)[3]; }

	inline Vec3p BoundMin() const { return VMin(a,VMin(b,c)); }
	inline Vec3p BoundMax() const { return VMax(a,VMax(b,c)); }

	template <class Vec>
	inline Vec Normal(const Vec&) const { return Vec(Nrm()); }

	template <class VecO,class Vec>
	typename Vec::TScalar Collide(const VecO &rOrig,const Vec &rDir) const;

	template <class Vec0,class Vec,class real>
	typename Vec::TScalar Barycentric(const Vec0 &rOrig,const Vec &rDir,real &u,real &v) const;

	int BeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL,float epsC,Vec3p *outCollisionPos=0) const;

	void SetFlag1(uint value) { ((uint*)&a)[3]=value; }
	void SetFlag2(uint value) { ((uint*)&b)[3]=value; }
	uint GetFlag1() const { return ((uint*)&a)[3]; }
	uint GetFlag2() const { return ((uint*)&b)[3]; }

private:
	void ComputeData() {
		Vec3p nrm=(b-a)^(c-a);
		nrm*=RSqrt(nrm|nrm);
		plane=Vec4p(nrm.x,nrm.y,nrm.z,nrm|a);
		e1ce2=(b-a)^(c-a);

		{ // invSize
			float e1=Length(b-a);
			float e2=Length(c-a);
			float ang=acos(((b-a)|(c-a))/(e1*e2));
			float size=sin(ang)*e1*e2*0.5f;
		//	float sizeSq=Min(e1,Min(e2,e3)); // surface area
			((float*)&c)[3]=Inv(size);	
		}
		EdgeNormals::ComputeEdgeNormals(this);
	}

public:
	Vec3p a,b,c;
	Vec4p plane;
	Vec3p e1ce2;
};

template <template <class> class EN> template <class VecO,class Vec>
typename Vec::TScalar TTriangle<EN>::Collide(const VecO &rOrig,const Vec &rDir) const {
	typedef typename Vec::TScalar real;
	typedef typename Vec::TBool Bool;

	real out=Const<real,-1>();

	real det = rDir|e1ce2;
	VecO tvec = rOrig-VecO(a);
	real u = rDir|(VecO(b-a)^tvec);
	real v = rDir|(tvec^VecO(c-a));
	Bool test=Min(u,v)>=Const<real,0>()&&u+v<=det;

	if (ForAny(test)) {
		real dist=-(tvec|Nrm())/(rDir|Nrm());
		out=Condition(test,dist,out);
	}

	return out;
}

template <template <class> class EN> template <class VecO,class Vec,class real>
typename Vec::TScalar TTriangle<EN>::Barycentric(const VecO &rOrig,const Vec &rDir,real &u,real &v) const {
	typedef typename Vec::TBool Bool;

	real det = rDir|e1ce2;
	VecO tvec = rOrig-VecO(a);
	real idet=Inv(det);
	u = (rDir|(VecO(b-a)^tvec))*idet;
	v = (rDir|(tvec^VecO(c-a)))*idet;
}


template <template <class> class EN>
int TTriangle<EN>::BeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL,float epsC,Vec3p *colPos) const {
	float dot=Abs(dir|Nrm());

	if(ForAny(Abs(dot)<=Const<float,1,100000>()))
		return 1;

	float idot=Inv(dot);

	float t=-((orig-a)|Nrm())*idot;
	Vec3p col=orig+dir*t;
	if(colPos) *colPos=col;

	if(ForAny(t<-epsC*idot)) return 0;

	float epsilon=(epsL*t+epsC)*idot;

	float dist[3]={
		(col-a)|Edge1Normal(),
		(col-b)|Edge2Normal(),
		(col-c)|Edge3Normal() };
	float min=Min(dist[0],Min(dist[1],dist[2]));

	return min<-epsilon?0:(min>epsilon?2:1);
}

typedef TTriangle<SlowEdgeNormals> Triangle;

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

