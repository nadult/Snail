#ifndef RTRACER_TRIANGLE_H
#define RTRACER_TRIANGLE_H

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
		Vec3p b,c;
		Convert(ta,a); Convert(tb,b); Convert(tc,c);
		ba=b-a; ca=c-a;
		ComputeData();
	}
	template <template <class> class T1>
	TTriangle(const TTriangle<T1> &other) {
		a=other.P1(); Vec3p b=other.P2(),c=other.P3();
		ba=b-a; ca=c-a;

		SetFlag1(other.GetFlag1());
		SetFlag2(other.GetFlag2());
		ComputeData();
	}
	TTriangle() {
	}

	inline Vec3p P1() const { return a; }
	inline Vec3p P2() const { return ba+a; }
	inline Vec3p P3() const { return ca+a; }

	inline Vec3p Edge1() const { return ba; }
	inline Vec3p Edge2() const { return ca-ba; }
	inline Vec3p Edge3() const { return -ca; }

	inline Vec3p Edge1Normal() const { return EdgeNormals::Edge1Normal(this); }
	inline Vec3p Edge2Normal() const { return EdgeNormals::Edge2Normal(this); }
	inline Vec3p Edge3Normal() const { return EdgeNormals::Edge3Normal(this); }

	inline Vec3p Nrm() const { return Vec3p(plane); }

	inline Vec3p BoundMin() const { return VMin(P1(),VMin(P2(),P3())); }
	inline Vec3p BoundMax() const { return VMax(P1(),VMax(P2(),P3())); }

	template <class Vec>
	inline Vec Normal(const Vec&) const { return Vec(Nrm()); }

	template <class VecO,class Vec>
	typename Vec::TScalar Collide(const VecO &rOrig,const Vec &rDir) const NOINLINE;

	template <class Vec0,class Vec,class real>
	void Barycentric(const Vec0 &rOrig,const Vec &rDir,real &u,real &v) const;

	int PrimaryBeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL) const;
	int BeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL,float epsC,Vec3p *outCollisionPos=0) const NOINLINE;

	void SetFlag1(uint value) { ((uint*)&a)[3]=value; }
	void SetFlag2(uint value) { ((uint*)&ba)[3]=value; }
	uint GetFlag1() const { return ((uint*)&a)[3]; }
	uint GetFlag2() const { return ((uint*)&ba)[3]; }

private:
	void ComputeData() {
		Vec3p nrm=(ba)^(ca);
		float e1ce2Len=Length(nrm);
		nrm/=e1ce2Len;
		((float*)&ca)[3]=e1ce2Len;
		plane=Vec4p(nrm.x,nrm.y,nrm.z,nrm|a);

		EdgeNormals::ComputeEdgeNormals(this);
		val1=ba|Edge2Normal();
		val2=ca|Edge3Normal();
	}

// BUT DONT EVEN THINK ABOUT MODIFYING IT :)
public:
	Vec3p a,ba,ca;
	Vec4p plane;
	float val1,val2;
	int dummy[2];
};

template <template <class> class EN> template <class VecO,class Vec>
typename Vec::TScalar TTriangle<EN>::Collide(const VecO &rOrig,const Vec &rDir) const {
	typedef typename Vec::TScalar real;
	typedef typename Vec::TBool Bool;

	real out=-1.0f;

	real det = rDir|Nrm();
	VecO tvec = rOrig-VecO(a);
	real u = rDir|(VecO(ba)^tvec);
	real v = rDir|(tvec^VecO(ca));
	Bool test=Min(u,v)>=0.0f&&u+v<=det*real(((float*)&ca)[3]);

	if (ForAny(test)) {
		real dist=-(tvec|Nrm())/det;
		out=Condition(test,dist,out);
	}

	return out;
}

template <template <class> class EN> template <class VecO,class Vec,class real>
void TTriangle<EN>::Barycentric(const VecO &rOrig,const Vec &rDir,real &u,real &v) const {
	typedef typename Vec::TBool Bool;

	real det = (rDir|Nrm())*((float*)&ca)[3];
	VecO tvec = rOrig-VecO(a);
	real idet=Inv(det);
	u = (rDir|(VecO(ba)^tvec))*idet;
	v = (rDir|(tvec^VecO(ca)))*idet;
}

template <template <class> class EN>
int TTriangle<EN>::PrimaryBeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL) const {
	float dot=dir.x*plane.x+dir.y*plane.y+dir.z*plane.z;
	Vec3f torig(orig.x-a.x,orig.y-a.y,orig.z-a.z);
	float t=torig.x*plane.x+torig.y*plane.y+torig.z*plane.z;

	if(dot<=0.0001f||t>=0.9999f) return 1;

	float idot=1.0f/dot;

	t=-t*idot;
	Vec3f col(torig.x+dir.x*t,torig.y+dir.y*t,torig.z+dir.z*t);

	const Vec3p e1=Edge1Normal(),e2=Edge2Normal(),e3=Edge3Normal();

	float distA=col.x*e1.x+col.y*e1.y+col.z*e1.z;
	float distB=col.x*e2.x+col.y*e2.y+col.z*e2.z-val1;
	float distC=col.x*e3.x+col.y*e3.y+col.z*e3.z-val2;
	float min=Min(distA,Min(distB,distC));
	float epsilon=epsL*t*idot;

	return min<-epsilon?0:(min>epsilon?2:1);
}

/*
template <template <class> class EN>
int TTriangle<EN>::PrimaryBeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL) const {
	float dot=Abs(dir|Nrm());

	if(ForAny(dot<=Const<float,1,10000>()))
		return 1;

	float idot=Inv(dot);
	Vec3p torig=orig-a;

	float t=-(torig|Nrm())*idot;
	Vec3p col=torig+dir*t;

	if(t<-idot) return 0;

	float epsilon=(epsL*t)*idot;

	float distA=col|Edge1Normal();
	float distB=(col-ba)|Edge2Normal();
	float distC=(col-ca)|Edge3Normal();
	float min=Min(distA,Min(distB,distC));

	return min<-epsilon?0:(min>epsilon?2:1);
}*/

template <template <class> class EN>
int TTriangle<EN>::BeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL,float epsC,Vec3p *colPos) const {
	float dot=Abs(dir|Nrm());

//	if(ForAny(dot<=Const<float,1,100000>()))
//		return 1;

	float idot=Inv(dot);

	float t=-((orig-a)|Nrm())*idot;
	Vec3f col=orig+dir*t;
	if(colPos) *colPos=col;

	if(t<-epsC*idot) return 0;

	float epsilon=(epsL*t+epsC)*idot;

	float distA=(col-P1())|Edge1Normal();
	float distB=(col-P2())|Edge2Normal();
	float distC=(col-P3())|Edge3Normal();
	float min=Min(distA,Min(distB,distC));

	return min<-epsilon?0:(min>epsilon?2:1);
}

typedef TTriangle<SlowEdgeNormals> Triangle;

typedef vector<TTriangle<FastEdgeNormals>,AlignedAllocator<TTriangle<FastEdgeNormals> > > TriVector;

#endif

