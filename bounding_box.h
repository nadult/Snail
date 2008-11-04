#ifndef RTRACER_BOUNDING_BOX_H
#define RTRACER_BOUNDING_BOX_H

#include "rtbase.h"

class BBox {
public:
	inline BBox() { }
	inline BBox(const Vec3f &tMin,const Vec3f &tMax) :min(tMin),max(tMax) { }
	BBox(const Vec3f *verts,uint count);
	BBox(const Vec3f *verts,uint count,const Matrix<Vec4f>&);
	
	const BBox &operator*=(const Matrix<Vec4f>&);
	const BBox &operator+=(const BBox &other);
	
	inline Vec3f Size() const { return max-min; }
	inline Vec3f Center() const { return (max+min)*0.5f; }
	
	template <class Real,class Vec>
	INLINE bool Test(const Vec &orig,const Vec &dir,const Real &maxDist=Real(1.0f/0.0f)) const {
		Real l1,l2,idir[3]={Inv(dir.x),Inv(dir.y),Inv(dir.z)};
		
		l1=idir[0]*(Real(min.x)-orig.x);
		l2=idir[0]*(Real(max.x)-orig.x);
		Real lmin=Min(l1,l2);
		Real lmax=Max(l1,l2);

		l1=idir[1]*(Real(min.y)-orig.y);
		l2=idir[1]*(Real(max.y)-orig.y);
		lmin=Max(Min(l1,l2),lmin);
		lmax=Min(Max(l1,l2),lmax);

		l1=idir[2]*(Real(min.z)-orig.z);
		l2=idir[2]*(Real(max.z)-orig.z);
		lmin=Max(Min(l1,l2),lmin);
		lmax=Min(Max(l1,l2),lmax);

		return !ForAll( lmax<Real(0.0f)||lmin>Min(lmax,maxDist) );
	}

	template <int size,class Real,class Vec>
	INLINE bool TestIP(const Vec &orig,const Vec *idir,const Real *__restrict__ maxDist) const {
		Real l1,l2;
		Real tx[2]={Real(min.x)-orig.x,Real(max.x)-orig.x};
		Real ty[2]={Real(min.y)-orig.y,Real(max.y)-orig.y};
		Real tz[2]={Real(min.z)-orig.z,Real(max.z)-orig.z};

		l1=idir[0].x*tx[0];
		l2=idir[0].x*tx[1];
		Real lmin=Min(l1,l2);
		Real lmax=Max(l1,l2);

		l1=idir[0].y*ty[0];
		l2=idir[0].y*ty[1];
		lmin=Max(Min(l1,l2),lmin);
		lmax=Min(Max(l1,l2),lmax);

		l1=idir[0].z*tz[0];
		l2=idir[0].z*tz[1];
		lmin=Max(Min(l1,l2),lmin);
		lmax=Min(Max(l1,l2),lmax);

		typename Vec::TBool mask=( lmax>=Real(0.0f)&&lmin<Min(lmax,maxDist[0]) );
		if(ForAny(mask)) return 1;
	
		for(int q=1;q<size;q++) {
			l1=idir[q].x*tx[0];
			l2=idir[q].x*tx[1];
			Real lmin=Min(l1,l2);
			Real lmax=Max(l1,l2);

			l1=idir[q].y*ty[0];
			l2=idir[q].y*ty[1];
			lmin=Max(Min(l1,l2),lmin);
			lmax=Min(Max(l1,l2),lmax);

			l1=idir[q].z*tz[0];
			l2=idir[q].z*tz[1];
			lmin=Max(Min(l1,l2),lmin);
			lmax=Min(Max(l1,l2),lmax);

			mask=mask||( lmax>=Real(0.0f)&&lmin<Min(lmax,maxDist[q]) );
			if(ForAny(mask)) return 1;
		}

		return 0;
	}

	template <class Real,class Vec>
	INLINE bool TestI(const Vec &orig,const Vec &vidir,const Real &maxDist=Real(1.0f/0.0f)) const {
		Real l1,l2,idir[3]={vidir.x,vidir.y,vidir.z};
		
		l1=idir[0]*(Real(min.x)-orig.x);
		l2=idir[0]*(Real(max.x)-orig.x);
		Real lmin=Min(l1,l2);
		Real lmax=Max(l1,l2);

		l1=idir[1]*(Real(min.y)-orig.y);
		l2=idir[1]*(Real(max.y)-orig.y);
		lmin=Max(Min(l1,l2),lmin);
		lmax=Min(Max(l1,l2),lmax);

		l1=idir[2]*(Real(min.z)-orig.z);
		l2=idir[2]*(Real(max.z)-orig.z);
		lmin=Max(Min(l1,l2),lmin);
		lmax=Min(Max(l1,l2),lmax);

		return !ForAll( lmax<Real(0.0f)||lmin>Min(lmax,maxDist) );
	}

	Vec3f min,max;
};

INLINE BBox operator+(const BBox &a,const BBox &b) { BBox out(a); out+=b; return out; }
INLINE BBox operator*(const BBox &a,const Matrix<Vec4f> &mat) { BBox out(a); out*=mat; return out; }

class OptBBox {
public:
	inline OptBBox() { }
	OptBBox(const BBox&,const Matrix<Vec4f>&);
	OptBBox(const BBox&,const Matrix<Vec4f>&,const Matrix<Vec4f>&inv);
	OptBBox(const Vec3f *verts,uint count);
	operator BBox() const;

	inline const Matrix<Vec4f> &GetTrans() const { return trans; }
	inline const BBox &GetBBox() const { return box; }
	
	template <class Real,class Vec>
	INLINE bool Test(const Vec &rayOrig,const Vec &rayDir,const Real &maxDist=Real(1.0f/0.0f)) const {
		return box.Test(invTrans*rayOrig,invTrans&rayDir,maxDist);
	}
	
	
private:
	BBox box;
	Matrix<Vec4f> trans,invTrans;
};



#endif
