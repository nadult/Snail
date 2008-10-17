#ifndef RTRACER_OBJECT_H
#define RTRACER_OBJECT_H

#include "rtbase.h"
#include "sphere.h"
#include "triangle.h"

class BIHTree;

class BBox {
public:
	BBox() { }
	BBox(const Vec3f &tMin,const Vec3f &tMax) :min(tMin),max(tMax) { }
	const BBox &operator*=(const Matrix<Vec4f> &m) {
		Vec3f v[8];
		v[0].x=v[2].x=v[4].x=v[6].x=min.x; v[1].x=v[3].x=v[5].x=v[7].x=max.x;
		v[0].y=v[1].y=v[4].y=v[5].y=min.y; v[3].y=v[2].y=v[6].y=v[7].y=max.y;
		v[0].z=v[1].z=v[2].z=v[3].z=min.z; v[4].z=v[5].z=v[6].z=v[7].z=max.z;

		const Vec3f &tv=v[0];
		min.x=max.x=m.x.x*tv.x+m.y.x*tv.y+m.z.x*tv.z;
		min.y=max.y=m.x.y*tv.x+m.y.y*tv.y+m.z.y*tv.z;
		min.z=max.z=m.x.z*tv.x+m.y.z*tv.y+m.z.z*tv.z;

		for(int n=1;n<8;n++) {
			const Vec3f &tv=v[n];
			float tx=m.x.x*tv.x+m.y.x*tv.y+m.z.x*tv.z;
			float ty=m.x.y*tv.x+m.y.y*tv.y+m.z.y*tv.z;
			float tz=m.x.z*tv.x+m.y.z*tv.y+m.z.z*tv.z;
			if(tx<min.x) min.x=tx; else if(tx>max.x) max.x=tx;
			if(ty<min.y) min.y=ty; else if(ty>max.y) max.y=ty;
			if(tz<min.z) min.z=tz; else if(tz>max.z) max.z=tz;
		}

		min.x+=m.w.x; max.x+=m.w.x;
		min.y+=m.w.y; max.y+=m.w.y;
		min.z+=m.w.z; max.z+=m.w.z;
			
		return *this;
	}
	
	Vec3f min,max;
};


class Object {
public:
	Object(const BIHTree*,const Matrix<Vec4f>&);
	
	const BIHTree *object;
	Matrix<Vec4f> trans,invTrans;
	BBox box;
};

/*
 
 
	Triangle
		TracingData = Position*3, Normal*3
		ShadingData = Normal*3, Color*3
	
	Sphere
		TracingData = 
	
	Nurbsy
	
	BIHTree
	
	BVHTree
	
  	
  
 */

/*
class Object
{
public:
	enum { MaxObjs=10000 };

	static Triangle tris[MaxObjs] __attribute__ ((aligned(16)));
	static Sphere spheres[MaxObjs] __attribute__ ((aligned(16)));
	static int nObjs;

public:
	Object(const Sphere &sp) :type(T_SPHERE) {
		if(nObjs==MaxObjs)
			throw Exception("Object buffer overflow");

		id=nObjs; nObjs++;
		spheres[id]=sp;
	}
	Object(const Triangle &tr) :type(T_TRIANGLE) {
		if(nObjs==MaxObjs)
			throw Exception("Object buffer overflow");

		id=nObjs; nObjs++;
		tris[id]=tr;
	}
	Object() { }
	~Object() {
	}

	u32 id;
	enum Type { T_SPHERE=1, T_TRIANGLE=2, };
	char type;

	// Object is complete within the node
//	bool fullInNode;
//	bool FullInNode() const { return 0; }
//	void SetFullInNode(bool full) { }

	template <class VecO,class Vec>
	INLINE typename Vec::TScalar Collide(const VecO &rOrig,const Vec &rDir) const {
		switch(type) {
		case T_SPHERE:
			return spheres[id].Collide(rOrig,rDir);
		case T_TRIANGLE:
			return tris[id].Collide(rOrig,rDir);
		}
	}

	// return 2 if every ray in beam collides with the object
	INLINE int BeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL,float epsC) const {
		switch(type) {
		case T_TRIANGLE:
			return tris[id].BeamCollide(orig,dir,epsL,epsC);
		default:
			return 1;
		}
	}

	template <class Vec>
	INLINE Vec Normal(const Vec &colPos) const {
		Vec out;
		switch(type) {
		case T_SPHERE: {
			Vec sPos(spheres[id].pos);
			Vec surfNormal=colPos-sPos;
			surfNormal*=RSqrt(surfNormal|surfNormal);
			out=surfNormal;
			break; }
		case T_TRIANGLE:
			out=Vec(tris[id].Nrm());
			break;
		}
		return out;
	}

	Vec3p BoundMin() const {
		Vec3p out;
		switch(type) {
		case T_SPHERE: out=spheres[id].BoundMin(); break;
		case T_TRIANGLE: out=spheres[id].BoundMin(); break;
		}
		return out;
	}
	Vec3p BoundMax() const {
		Vec3p out;
		switch(type) {
		case T_SPHERE: out=spheres[id].BoundMax(); break;
		case T_TRIANGLE: out=spheres[id].BoundMax(); break;
		}
		return out;
	}
};
*/

#endif
