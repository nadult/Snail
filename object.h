#ifndef RTRACER_OBJECT_H
#define RTRACER_OBJECT_H

#include "rtbase.h"
#include "sphere.h"
#include "triangle.h"

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

