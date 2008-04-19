#ifndef OBJECT_H
#define OBJECT_H

#include "rtbase.h"
#include "sphere.h"
#include "triangle.h"

class Object
{
public:
	enum { MaxObjs=1000000 };

	static Triangle tris[MaxObjs] __attribute__ ((aligned(16)));
//	static Sphere spheres[MaxObjs] __attribute__ ((aligned(16)));
	static Vec3p bounds[MaxObjs*2];
	static int nObjs;

public:
/*	Object(const Sphere &sp) :type(T_SPHERE),fullInNode(0) {
		if(nObjs==MaxObjs)
			throw Exception("Object buffer overflow");

		id=nObjs; nObjs++;
		spheres[id]=sp;
		bounds[id*2+0]=sp.pos-Vec3p(sp.rad);
		bounds[id*2+1]=sp.pos+Vec3p(sp.rad);
		bid=id;
	}*/
	Object(const Triangle &tr) /*:type(T_TRIANGLE),fullInNode(0)*/ {
		if(nObjs==MaxObjs)
			throw Exception("Object buffer overflow");

		id=nObjs; nObjs++;
		tris[id]=tr;
		bounds[id*2+0]=VMin(tr.a,VMin(tr.b,tr.c));
		bounds[id*2+1]=VMax(tr.a,VMax(tr.b,tr.c));
		bid=id;
	}
	Object() { }
	~Object() {
	}

	u32 id,bid;
//	enum Type { T_SPHERE=1, T_TRIANGLE=2, };
//	char type;

	// Object is complete within the node
//	bool fullInNode;

	bool FullInNode() const { return 0; }
	void SetFullInNode(bool full) { }

	template <class VecO,class Vec>
	INLINE typename Vec::TScalar Collide(const VecO &rOrig,const Vec &rDir) const {
//		switch(type) {
//		case T_SPHERE:
//			return spheres[id].Collide(rOrig,rDir);
//		case T_TRIANGLE:
			return tris[id].Collide(rOrig,rDir);
//		}
	}

	// return 2 if every ray in beam collides with the object
	INLINE int BeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL,float epsC) const {
//		switch(type) {
//		case T_TRIANGLE:
			return tris[id].BeamCollide(orig,dir,epsL,epsC);
//		default:
//			return 1;
//		}
	}
//	template <class Vec0,class Vec>
//		INLINE typename bool Collide(
	template <class Vec>
	INLINE Vec Normal(const Vec &colPos) const {
		Vec out;
	//	switch(type) {
	//	case T_SPHERE: {
	//		Vec sPos(spheres[id].pos);
	//		Vec surfNormal=colPos-sPos;
	//		surfNormal*=RSqrt(surfNormal|surfNormal);
	//		out=surfNormal;
	//		break; }
	//	case T_TRIANGLE:
			out=Vec(tris[id].Nrm());
	//		break;
	//	}
		return out;
	}

	const Vec3p &BoundMin() const {
		return bounds[bid*2+0];
	}
	const Vec3p &BoundMax() const {
		return bounds[bid*2+1];
	}
};

#endif

