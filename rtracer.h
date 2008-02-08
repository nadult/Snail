#ifndef RTRACER_H
#define RTRACER_H

#include <cassert>
#include <vector>
#include <exception>
#include <string>
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
typedef SSEPReal	floatp;
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

template <class Vec>
INLINE void GetVecSigns(const Vec &dir,int *dSign)
{
	dSign[0]=SignMask(dir.X());
	dSign[1]=SignMask(dir.Y());
	dSign[2]=SignMask(dir.Z());

	// Wszystkie wektorki maja takie same znaki kierunk�
//	assert(dSign[0]==15||dSign[0]==0);
//	assert(dSign[1]==15||dSign[1]==0);
//	assert(dSign[2]==15||dSign[2]==0);

	dSign[0]=dSign[0]?0:1;
	dSign[1]=dSign[1]?0:1;
	dSign[2]=dSign[2]?0:1;
}

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
	floatp radp2,rad;
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

	void ComputeData() {
		nrm=(b-a)^(c-a);
		nrm*=RSqrt(nrm|nrm);
		dist=Vec3q(nrm)|a;
		edgeNrm[0]=(b-a)^nrm;
		edgeNrm[1]=(c-b)^nrm;
		edgeNrm[2]=(a-c)^nrm;
	}
	Vec3p a,b,c;
	Vec3p edgeNrm[3];
	Vec3p nrm;
	floatq dist;
};

template <class VecO,class Vec>
typename Vec::TScalar Sphere::Collide(const VecO &rOrigin,const Vec &rDir) const
{
	typedef typename Vec::TScalar base;

	VecO dst=rOrigin-pos;
	base b=rDir|dst,d=b*b-((dst|dst)-radp2);

	typename Vec::TBool mask=d>=Const<base,0>::Value();
	return ForAny(mask)?Condition(mask,-b-Sqrt(d),Const<base,-1>::Value()):Const<base,-1>::Value();
}

template <class VecO,class Vec>
typename Vec::TScalar Triangle::Collide(const VecO &rOrig,const Vec &rDir) const
{
	typedef typename Vec::TScalar base;
	typedef typename Vec::TBool Bool;

	base dist=-((rOrig-a)|nrm)/(rDir|nrm);
	base out=Const<base,-1>::Value();

	Bool test=dist>Const<base,1,1000>::Value();
	if(ForAny(test)) {
		Vec H=Vec(rOrig)+rDir*dist;

		test.m=(_mm_and_ps(
			_mm_and_ps(	(Vec(edgeNrm[0])|(H-a)).m,
						(Vec(edgeNrm[1])|(H-b)).m),
						(Vec(edgeNrm[2])|(H-c)).m)<Const<base,0>::Value()).m;


		/*test=test&&(Vec(edgeNrm[0])|(H-a))<Const<base,0>::Value();
		test=test&&(Vec(edgeNrm[1])|(H-b))<Const<base,0>::Value();
		test=test&&(Vec(edgeNrm[2])|(H-c))<Const<base,0>::Value();*/
		
		out=Condition(test,dist,out);
	}

	return out;
/*
	Vec3p eb=c-a,ec=b-a;
	Vec3p anrm=Abs(nrm);
	Vec H=rOrig+rDir*dist;

	int k;
	if (ForAny(anrm.X()>anrm.Y()))
		if (ForAny(anrm.X()>anrm.Z())) k=0; else k=2;
	else
		if (ForAny(anrm.Y()>anrm.Z())) k=1; else k=2;
	base beta,gamma;

	switch(k) {
	case 0:
		beta=(eb.Y() * H.Z() - eb.Z() * H.Y()) / (eb.Y() * ec.Z() - eb.Z() * ec.Y());
		gamma=(ec.Z() * H.Y() - ec.Y() * H.Z()) / (eb.Y() * ec.Z() - eb.Z() * ec.Y());
		break;
	case 1:
		beta=(eb.Z() * H.X() - eb.X() * H.Z()) / (eb.Z() * ec.X() - eb.X() * ec.Z());
		gamma=(ec.X() * H.Z() - ec.Z() * H.X()) / (eb.Z() * ec.X() - eb.X() * ec.Z());
		break;
	case 2:
		beta=(eb.X() * H.Y() - eb.Y() * H.X()) / (eb.X() * ec.Y() - eb.Y() * ec.X());
		gamma=(ec.Y() * H.X() - ec.X() * H.Y()) / (eb.X() * ec.Y() - eb.Y() * ec.X());
		break;
	}

	return Condition(dist>ConstEpsilon<base>::Value()&&beta>=Const<base,0>::Value()&&
					gamma>=Const<base,0>::Value()&&beta+gamma<=Const<base,1>::Value(),
					dist,Const<base,-1>::Value());*/
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
	enum { MaxObjs=100000 };

	static Triangle tris[MaxObjs];
	static Sphere spheres[MaxObjs];
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
		case T_TRIANGLE: {
			out=Vec(tris[id].nrm); }
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

void LoadModel(const char *fileName,vector<Object> &out,float scale);


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
	floatp zeroDist;
};


class Image
{
public:
	Image(size_t w,size_t h);
	void SaveToFile(const char*fileName);
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
