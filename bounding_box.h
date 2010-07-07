#ifndef RTRACER_BOUNDING_BOX_H
#define RTRACER_BOUNDING_BOX_H

#include "rtbase.h"

template <bool,bool> class RayGroup;

class CornerRays;
class RayInterval;

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
	float Width() const { return max.x-min.x; }
	float Height() const { return max.y-min.y; }
	float Depth() const { return max.z-min.z; }
	float SurfaceArea() const {
		Vec3f size = Size();
		return 2.0f * (size.x * (size.y + size.z) + size.y * size.z);
	}
	
	bool Contains(const BBox &rhs) const {
		return 	min.x <= rhs.min.x && min.y <= rhs.min.y && min.z <= rhs.min.z &&
				max.x >= rhs.max.x && max.y >= rhs.max.y && max.z >= rhs.max.z;
	}
	bool Contains(const BBox &rhs,float t) const {
		return BBox(Center()-Size()*0.5f*t,Center()+Size()*0.5f*t).Contains(rhs);
	}
	
	const Vec3f ClosestPoint(const Vec3f &point) const {
		return Vec3f(
			point.x < min.x ? min.x : point.x > max.x ? max.x : point.x,
			point.y < min.y ? min.y : point.y > max.y ? max.y : point.y,
			point.z < min.z ? min.z : point.z > max.z ? max.z : point.z );
	}

	void UpdateMinMaxDist(const float* __restrict__ orig,const float* minInv,const float *maxInv,
								 int dirMask,float& __restrict__ tMin,float & __restrict__ tMax) const {
		Vec3f origin(orig[0],orig[1],orig[2]);
		Vec3f ttMin=(min-origin)*Vec3f(minInv[0],minInv[1],minInv[2]);
		Vec3f ttMax=(max-origin)*Vec3f(maxInv[0],maxInv[1],maxInv[2]);

		if(dirMask&1) Swap(ttMin.x,ttMax.x);
		if(dirMask&2) Swap(ttMin.y,ttMax.y);
		if(dirMask&4) Swap(ttMin.z,ttMax.z);

		tMax=Min(Min(ttMax.x,ttMax.y),tMax);
		tMax=Min(ttMax.z,tMax);
			
		tMin=Max(Max(ttMin.x,ttMin.y),tMin);
		tMin=Max(ttMin.z,tMin);
	}

	template <int size,bool sharedOrigin>
	void UpdateMinMaxDist(	const RayGroup<size,sharedOrigin> &rays,int dirMask,
							floatq *__restrict__ tMin,floatq *__restrict__ tMax) const {
		Vec3p rMin = min, rMax = max;

		if(dirMask&1) Swap(rMin.x,rMax.x);
		if(dirMask&2) Swap(rMin.y,rMax.y);
		if(dirMask&4) Swap(rMin.z,rMax.z);

		if(sharedOrigin) {
			Vec3q rrMin=Vec3q(rMin)-rays.Origin(0),rrMax=Vec3q(rMax)-rays.Origin(0);

			for(int q=0;q<size;q++) {
				Vec3q ttMin=rrMin*rays.IDir(q);
				Vec3q ttMax=rrMax*rays.IDir(q);
			
				tMax[q]=Min(Min(tMax[q],ttMax.x),Min(ttMax.y,ttMax.z));
				tMin[q]=Max(Max(tMin[q],ttMin.x),Max(ttMin.y,ttMin.z));
			}
		}
		else {
			for(int q=0;q<size;q++) {
				Vec3q ttMin=(Vec3q(rMin)-rays.Origin(q))*rays.IDir(q);
				Vec3q ttMax=(Vec3q(rMax)-rays.Origin(q))*rays.IDir(q);
				
				tMax[q]=Min(Min(tMax[q],ttMax.x),Min(ttMax.y,ttMax.z));
				tMin[q]=Max(Max(tMin[q],ttMin.x),Max(ttMin.y,ttMin.z));
			}
		}
	}

	template <class Real,class Vec>
	bool Test(const Vec &orig,const Vec &dir,const Real &maxDist=Real(1.0f/0.0f)) const {
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
	bool TestIP(const Vec &orig,const Vec *__restrict__ idir,const Real *__restrict__ maxDist) const {
		Real l1,l2;
		Real tx[2]={Real(min.x)-orig.x,Real(max.x)-orig.x};
		Real ty[2]={Real(min.y)-orig.y,Real(max.y)-orig.y};
		Real tz[2]={Real(min.z)-orig.z,Real(max.z)-orig.z};

/*		Real l3,l4; Vec3q min=idir[0],max=idir[0];
		for(int n=1;n<size;n++) {
			min=VMin(min,idir[n]);
			max=VMax(max,idir[n]);
		}

		l1=min.x*tx[0]; l3=max.x*tx[0];
		l2=min.x*tx[1]; l4=max.x*tx[1];
		Real lmin=Min(Min(l1,l2),Min(l3,l4));
		Real lmax=Max(Max(l1,l2),Max(l3,l4));
		
		l1=min.y*ty[0]; l3=max.y*ty[0];
		l2=min.y*ty[1]; l4=max.y*ty[1];
		lmin=Min(lmin,Min(Min(l1,l2),Min(l3,l4)));
		lmax=Max(lmax,Max(Max(l1,l2),Max(l3,l4)));
		
		l1=min.z*tz[0]; l3=max.z*tz[0];
		l2=min.z*tz[1]; l4=max.z*tz[1];
		lmin=Min(lmin,Min(Min(l1,l2),Min(l3,l4)));
		lmax=Max(lmax,Max(Max(l1,l2),Max(l3,l4)));

		return ForAny(lmax>=Real(0.0f)&&lmin<Min(lmax,maxDist[0])); */

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
	bool TestI(Vec orig, Vec idir, Real maxDist = Real(constant::inf)) const {
		Real l1 = idir.x * (Real(min.x) - orig.x);
		Real l2 = idir.x * (Real(max.x) - orig.x);
		Real lmin = Min(l1, l2);
		Real lmax = Max(l1, l2);

		l1 = idir.y * (Real(min.y) - orig.y);
		l2 = idir.y * (Real(max.y) - orig.y);
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		l1 = idir.z * (Real(min.z) - orig.z);
		l2 = idir.z * (Real(max.z) - orig.z);
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		return !ForAll( lmax < Real(0.0f) || lmin > Min(lmax, maxDist) );
	}
	template <class Real,class Vec>
	bool TestI(Vec orig, Vec idir, Real maxDist, f32x4b mask) const {
		Real l1 = idir.x * (Real(min.x) - orig.x);
		Real l2 = idir.x * (Real(max.x) - orig.x);
		Real lmin = Min(l1, l2);
		Real lmax = Max(l1, l2);

		l1 = idir.y * (Real(min.y) - orig.y);
		l2 = idir.y * (Real(max.y) - orig.y);
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		l1 = idir.z * (Real(min.z) - orig.z);
		l2 = idir.z * (Real(max.z) - orig.z);
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		return !ForAll( lmax < Real(0.0f) || lmin > Min(lmax, maxDist) || !mask );
	}


	bool TestInterval(const RayInterval&) const;

	Vec3f min, max;
};

SERIALIZE_AS_POD(BBox)

float BoxPointDistanceSq(const BBox &box,const Vec3f &point);

inline BBox operator+(const BBox &a,const BBox &b) { BBox out(a); out+=b; return out; }
inline BBox operator*(const BBox &a,const Matrix<Vec4f> &mat) { BBox out(a); out*=mat; return out; }

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
	bool Test(const Vec &rayOrig,const Vec &rayDir,const Real &maxDist=Real(1.0f/0.0f)) const {
		return box.Test(invTrans*rayOrig,invTrans&rayDir,maxDist);
	}
	
	
private:
	BBox box;
	Matrix<Vec4f> trans,invTrans;
};



#endif
