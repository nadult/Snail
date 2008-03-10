#ifndef SPHERE_H
#define SPHERE_H

#include "rtbase.h"

class Sphere
{
public:
	Sphere() { }
	Sphere(const Vec3f &p,float r) {
		Convert(p,pos);
		Convert(r,rad);
		ComputeData();
	}
	void ComputeData() {
		radp2=rad*rad;
	}

	template <class VecO,class Vec>
	typename Vec::TScalar Collide(const VecO &rOrig,const Vec &rDir) const;

	Vec3p pos;
	float radp2,rad;
};

template <class VecO,class Vec>
INLINE typename Vec::TScalar Sphere::Collide(const VecO &rOrigin,const Vec &rDir) const
{
	typedef typename Vec::TScalar base;

	VecO dst=rOrigin-pos;
	base b=rDir|dst,d=b*b-((dst|dst)-radp2);

	typename Vec::TBool mask=d>=Const<base,0>();
	return ForAny(mask)?Condition(mask,-b-Sqrt(d),Const<base,-1>()):Const<base,-1>();
}


#endif

