#ifndef RTRACER_TRIANGLE_H
#define RTRACER_TRIANGLE_H

#include "rtbase.h"
#include "ray_group.h"
#include "context.h"

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
	Vec3p dummy;
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
	enum { isctFlags=isct::fDistance };

	typedef TTriangle BaseElement;
	enum { isComplex=0 }; // doesn't contain any hierarchy of objects
	typedef TEdgeNormals< TTriangle<TEdgeNormals> > EdgeNormals;

	const TTriangle &GetElement(int) const { return *this; }

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

	bool Test() const {
		bool nan=isnan(a.x)||isnan(a.y)||isnan(a.z);
		nan=nan||isnan(ba.x)||isnan(ba.y)||isnan(ba.z);
		nan=nan||isnan(ca.x)||isnan(ca.y)||isnan(ca.z);
		nan=nan||isnan(plane.x)||isnan(plane.y)||isnan(plane.z);

		return !nan;
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
	INLINE Vec3p Nrm(int) const { return Nrm(); }

	inline Vec3p BoundMin() const { return VMin(P1(),VMin(P2(),P3())); }
	inline Vec3p BoundMax() const { return VMax(P1(),VMax(P2(),P3())); }

	template <class Vec>
	inline Vec Normal(const Vec&) const { return Vec(Nrm()); }

	template <int flags,class VecO,class Vec>
	Isct<typename Vec::TScalar,1,isct::fDistance|flags>
		Collide(const VecO &rOrig,const Vec &rDir) const NOINLINE;

	template <int flags,class VecO,class Vec>
	INLINE Isct<typename Vec::TScalar,1,isct::fDistance|flags>
		Collide(const VecO &rOrig,const Vec &rDir,float maxDist) const
		{ return Collide<flags>(rOrig,rDir); }

	template <int flags,int packetSize>
	Isct<f32x4,packetSize,isct::fDistance|flags>
		Collide(const RayGroup<packetSize,flags> &rays) const NOINLINE;

	template <class Vec0,class Vec>
	Vec3<typename Vec::TScalar> Barycentric(const Vec0 &rOrig,const Vec &rDir,int) const;

	int PrimaryBeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL) const;
	int BeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL) const;

	void SetFlag1(uint value) { a.t0=UValue(value).f; }
	void SetFlag2(uint value) { ba.t0=UValue(value).f; }
	uint GetFlag1() const { return UValue(a.t0).i; }
	uint GetFlag2() const { return UValue(ba.t0).i; }

private:
	void ComputeData() {
		Vec3p nrm=(ba)^(ca);
		float e1ce2Len=Length(nrm);
		nrm/=e1ce2Len;
		ca.t0=e1ce2Len;
		plane=Vec4p(nrm.x,nrm.y,nrm.z,nrm|a);

		EdgeNormals::ComputeEdgeNormals(this);
	}

// BUT DONT EVEN THINK ABOUT MODIFYING IT :)
public:
	Vec3p a,ba,ca;
	Vec4p plane;
};

template <template <class> class EN> template <int flags,class VecO,class Vec>
Isct<typename Vec::TScalar,1,isct::fDistance|flags> TTriangle<EN>::Collide(const VecO &rOrig,const Vec &rDir) const {
	typedef typename Vec::TScalar real;
	typedef typename Vec::TBool Bool;

	Isct<typename Vec::TScalar,1,isct::fDistance|flags> out;

	real det = rDir|Nrm();
	VecO tvec = rOrig-VecO(a);
	real u = rDir|(VecO(ba)^tvec);
	real v = rDir|(tvec^VecO(ca));
	Bool test=Min(u,v)>=0.0f&&u+v<=det*real(ca.t0);

//	if (ForAny(test)) {
		real dist=-(tvec|Nrm())/det;
		out.Distance()=Condition(test,dist,real(1.0f/0.0f));
//	}

	return out;
}

template<template<class> class EN> template <int flags,int packetSize>
Isct<f32x4,packetSize,isct::fDistance|flags> TTriangle<EN>::Collide(const RayGroup<packetSize,flags> &rays) const {
	Isct<f32x4,packetSize,isct::fDistance|flags> out;
	Vec3p nrm=Nrm();
	Vec3q ta(a);

	Vec3q sharedTVec;
	if(flags&isct::fShOrig) sharedTVec=rays.Origin(0)-ta;
	f32x4 infinity=1.0f/0.0f;

	for(int q=0;q<packetSize;q++) {
		floatq det=rays.Dir(q)|nrm;
		Vec3q tvec=flags&isct::fShOrig?sharedTVec:rays.Origin(q)-ta;

		floatq u=rays.Dir(q)|(Vec3q(ba)^tvec);
		floatq v=rays.Dir(q)|(tvec^Vec3q(ca));
		f32x4b test=Min(u,v)>=0.0f&&u+v<=det*floatq(ca.t0);

	//	if(ForAny(test)) {
			floatq dist=-(tvec|nrm)/det;
			out.Distance(q)=Condition(test,dist,infinity);
	//	}
	}

	return out;
}

template <template <class> class EN> template <class VecO,class Vec>
Vec3<typename Vec::TScalar> TTriangle<EN>::Barycentric(const VecO &rOrig,const Vec &rDir,int) const {
	typedef typename Vec::TBool Bool;
	Vec3<typename Vec::TScalar> out;

	typename Vec::TScalar det = (rDir|Nrm())*ca.t0;
	VecO tvec = rOrig-VecO(a);
	typename Vec::TScalar idet=Inv(det);
	out.z = (rDir|(VecO(ba)^tvec))*idet;
	out.y = (rDir|(tvec^VecO(ca)))*idet;
	out.x=(typename Vec::TScalar)(1.0f)-out.y-out.z;

	return out;
}

template <template <class> class EN>
int TTriangle<EN>::BeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL) const {
	float dot=dir|Nrm();

	float t=((orig-a)|Nrm());
	if(t>0.0f||dot<0.0f) return 0;

	float idot=Inv(dot);
	t=-t*idot;

	Vec3p col=orig+dir*t;
	float epsilon=(epsL*t)*idot;

	float distA=(col-P1())|Edge1Normal();
	float distB=(col-P2())|Edge2Normal();
	float distC=(col-P3())|Edge3Normal();
	float min=Min(distA,Min(distB,distC));

	return min<-epsilon?0:(min>epsilon?2:1);
}

typedef TTriangle<SlowEdgeNormals> Triangle;

typedef vector<Triangle,AlignedAllocator<Triangle> > TriVector;


class ShTriangle {
public:
	Vec2f uv[3];
	Vec3f nrm[3];
	int dummy;

//	Vec3p normal,tangent,binormal;

	ShTriangle(const Vec3f &p1,const Vec3p &p2,const Vec3f &p3,const Vec2f &uv1,const Vec2f &uv2,const Vec2f &uv3,
				const Vec3f &nrm1,const Vec3f &nrm2,const Vec3f &nrm3) {
		Vec3p pos[3];

		pos[0]=p1; pos[1]=p2; pos[2]=p3;
		uv[0]=uv1; uv[1]=uv2; uv[2]=uv3;
		nrm[0]=nrm1; nrm[1]=nrm2; nrm[2]=nrm3;
/*		normal=(pos[1]-pos[0])^(pos[2]-pos[0]);
		normal*=RSqrt(normal|normal);
		
		Vec3p side0=pos[0]-pos[1];
		Vec3p side1=pos[2]-pos[0];

		float deltaV0=uv[0].y-uv[1].y;
		float deltaV1=uv[2].y-uv[0].y;
		tangent=side0*deltaV1-side1*deltaV0;
		tangent*=RSqrt(tangent|tangent);

		float deltaU0 = uv[0].x-uv[1].x;
		float deltaU1 = uv[2].x-uv[0].x;
		binormal=side0*deltaU1-side1*deltaU0;
		binormal*=RSqrt(binormal|binormal);

		///Now, take the cross product of the tangents to get a vector which 
		///should point in the same direction as our normal calculated above. 
		///If it points in the opposite direction (the dot product between the normals is less than zero), 
		///then we need to reverse the s and t tangents. 
		///This is because the triangle has been mirrored when going from tangent space to object space.
		///reverse tangents if necessary
		Vec3p tangentCross=tangent^binormal;
		if ((tangentCross|normal)<0.0f) {
			tangent=-tangent;
			binormal=-binormal;
		} */
	}
};

static_assert(sizeof(ShTriangle)==64,"sizeof(ShTriangle)!=64");

typedef vector<ShTriangle,AlignedAllocator<ShTriangle> > ShTriVector;

#endif

