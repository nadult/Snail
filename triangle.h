#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "rtbase.h"


class Triangle
{
public:
	Triangle(const Vec3f &ta,const Vec3f &tb,const Vec3f &tc) {
		Convert(ta,a); Convert(tb,b); Convert(tc,c);
		ComputeData();
	}
	Triangle() {
	}

	template <class VecO,class Vec>
	typename Vec::TScalar Collide(const VecO &rOrig,const Vec &rDir) const;

	int BeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL,float epsC) const;

	void ComputeData() {
		Vec3p nrm=(b-a)^(c-a);
		nrm*=RSqrt(nrm|nrm);
		plane=Vec4p(nrm.x,nrm.y,nrm.z,nrm|a);
		e1ce2=(b-a)^(c-a);
		e1n=nrm^(b-a);
		e2n=nrm^(c-b);
		e3n=nrm^(a-c);
		e1n*=RSqrt(e1n|e1n);
		e2n*=RSqrt(e2n|e2n);
		e3n*=RSqrt(e3n|e3n);
	}
	Vec3p Nrm() const { return Vec3p(plane); }

	Vec3p a,b,c;
	Vec4p plane;
	Vec3p e1ce2,e1n,e2n,e3n;
};

template <class VecO,class Vec>
INLINE typename Vec::TScalar Triangle::Collide(const VecO &rOrig,const Vec &rDir) const
{
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

INLINE int Triangle::BeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL,float epsC) const
{
	float dot=dir|Nrm();

	if(ForAny(dot<=Const<float,1,100>()))
		return 1;

	float idot=Inv(dir|Nrm());

	float t=-((orig-a)|Nrm())*idot;
	Vec3p col=orig+dir*t;

	if(ForAny(t<-epsC*idot)) return 0;

	float epsilon=(epsL*t+epsC)*idot;

	float dist[3]={(col-a)|e1n,(col-b)|e2n,(col-c)|e3n};
	float min=Min(dist[0],Min(dist[1],dist[2]));

	if(min<-epsilon) return 0;
	return (min>epsilon?2:1);
}

#endif

