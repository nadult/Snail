#ifndef RTRACER_H
#define RTRACER_H

#include <cassert>
#include <vector>
#include <exception>
#include <string>
#include <memory.h>
#include "omp.h"
#include "veclib.h"


using namespace veclib;

typedef Vec2<float> Vec2f;
typedef Vec3<float> Vec3f;
typedef Vec4<float> Vec4f;
typedef SSEPVec2	Vec2p;
typedef SSEPVec3	Vec3p;
typedef SSEPVec4	Vec4p;
typedef SSEVec2		Vec2q;
typedef SSEVec3		Vec3q;
typedef SSEVec4		Vec4q;
typedef SSEReal		floatq;

using std::vector;
using std::pair;
using std::string;

class Exception: public std::exception
{
	string data;
public:
	Exception(const string &txt) :data(txt) { }
	Exception(const char *txt) :data(txt) { }
	~Exception() throw() { }
	const char *what() const throw() { return data.c_str(); }
};

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

	bool BeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL,float epsC) const;

	void ComputeData() {
		Vec3p nrm=(b-a)^(c-a);
		nrm*=RSqrt(nrm|nrm);
		plane=Vec4p(nrm.X(),nrm.Y(),nrm.Z(),nrm|a);
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
typename Vec::TScalar Sphere::Collide(const VecO &rOrigin,const Vec &rDir) const
{
	typedef typename Vec::TScalar base;

	VecO dst=rOrigin-pos;
	base b=rDir|dst,d=b*b-((dst|dst)-radp2);

	typename Vec::TBool mask=d>=Const<base,0>();
	return ForAny(mask)?Condition(mask,-b-Sqrt(d),Const<base,-1>()):Const<base,-1>();
}

template <class VecO,class Vec>
typename Vec::TScalar Triangle::Collide(const VecO &rOrig,const Vec &rDir) const
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

INLINE bool Triangle::BeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL,float epsC) const
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
	if(ForAny(Min(dist[0],Min(dist[1],dist[2]))<-epsilon)) return 0;

	return 1;
}


/*
class Plane
{
public:
	Plane(const Vec3f &n,const float &d) :nrm(n),dist(d) {
	}
	Plane() {
	}

	template <class Vec>
	typename Vec::TScalar Collide(

	Vec3f nrm;
	float dist;
}; */

class Object
{
public:
	enum { MaxObjs=1000000 };

	static Triangle tris[MaxObjs] __attribute__ ((aligned(16)));
	static Sphere spheres[MaxObjs] __attribute__ ((aligned(16)));
	static Vec3p bounds[MaxObjs*2];
	static int nObjs;

public:
	Object(const Sphere &sp) :type(T_SPHERE),fullInNode(0),lastVisit(-1) {
		if(nObjs==MaxObjs)
			exit(0);//throw exception("Object buffer overflow");

		id=nObjs; nObjs++;
		spheres[id]=sp;
		bounds[id*2+0]=sp.pos-Vec3p(sp.rad);
		bounds[id*2+1]=sp.pos+Vec3p(sp.rad);
	}
	Object(const Triangle &tr) :type(T_TRIANGLE),fullInNode(0),lastVisit(-1) {
		if(nObjs==MaxObjs)
			exit(0);//throw exception("Object buffer overflow");

		id=nObjs; nObjs++;
		tris[id]=tr;
		bounds[id*2+0]=Min(tr.a,Min(tr.b,tr.c));
		bounds[id*2+1]=Max(tr.a,Max(tr.b,tr.c));
	}
	~Object() {
	}

	mutable int lastVisit;
	u32 id;
	enum Type {
		T_SPHERE = 1,
		T_TRIANGLE = 2,
	} type;

	// Object is complete within the node
	bool fullInNode;

	template <class VecO,class Vec>
	INLINE typename Vec::TScalar Collide(const VecO &rOrig,const Vec &rDir) const {
		switch(type) {
		case T_SPHERE:
			return spheres[id].Collide(rOrig,rDir);
		case T_TRIANGLE:
			return tris[id].Collide(rOrig,rDir);
		}
	}
	INLINE bool BeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL,float epsC) const {
		switch(type) {
		case T_TRIANGLE:
			return tris[id].BeamCollide(orig,dir,epsL,epsC);
		default:
			return 1;
		}
	}
//	template <class Vec0,class Vec>
//		INLINE typename bool Collide(
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

	const Vec3p &BoundMin() const {
		return bounds[id*2+0];
	}
	const Vec3p &BoundMax() const {
		return bounds[id*2+1];
	}
};


void LoadWavefrontObj(const char *fileName,vector<Object> &out,float scale);
void LoadRaw(const char *fileName,vector<Object> &out,float scale);

class Material
{
public:
	float refl;
};

class Light
{
public:
	Light() { }
	Light(Vec3f p,Vec3f c) {
		Convert(p,pos); Convert(c,color);
		zeroDist=Sqrt(Max(color.X(),Max(color.Y(),color.Z())))*256.0f;
	}

	Vec3p pos,color;
	float zeroDist;
};


class Image
{
public:
	Image();
	Image(size_t w,size_t h);
	void SaveToFile(const char*fileName);
	void LoadFromFile(const char *fileName);
	void Pixel(int x,int y,char r,char g,char b);

	vector<char> buffer;
	size_t width,height;
};

class Camera
{
public:
	Camera() :pos(0,0,0),right(1,0,0),up(0,1,0),front(0,0,1),plane_dist(0.5f)
		{ }

	Vec3f pos,right,front,up;
	float plane_dist;
};

class SDLOutput
{
public:
	SDLOutput(int w,int h,bool full);
	virtual ~SDLOutput();

	virtual void Render(const Image &img);
	virtual bool PollEvents();
	virtual bool TestKey(u8 c);
	virtual int MouseDX();
	virtual int MouseDY();
};

#include "kdtree.h"

#endif

