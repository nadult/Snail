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

	bool BeamCollide(const floatq *pv,const floatq *ov,const Vec3p &negMask,const Vec3p&,const Vec3p&)
		const;

	void ComputeData() {
		nrm=(b-a)^(c-a);
		nrm*=RSqrt(nrm|nrm);
		dist=Vec3q(nrm)|a;
		e1=(b-a); e2=(c-a);
		e1ce2=e1^e2;
	}
	Vec3p a,b,c,e1,e2,e1ce2;
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

	base out=Const<base,-1>::Value();

	base det = rDir|e1ce2;
	VecO tvec = rOrig-VecO(a);
	base u = rDir|(VecO(e1)^tvec);
	base v = rDir|(tvec^VecO(e2));
	Bool test=Min(u,v)>=Const<base,0>::Value()&&u+v<=det;
	if (ForAny(test)) {
		base dist=-(tvec|nrm)*Inv(rDir|nrm);
		out=Condition(test,dist,out);
	}

	return out;
}

INLINE bool Triangle::BeamCollide(const floatq *pv,const floatq *ov,const Vec3p &negMask,
									const Vec3p &nodeMin,const Vec3p &nodeMax) const
{
//	int out[3]={
//		(_mm_movemask_ps(_mm_cmplt_ps(a.m,nodeMin.m))+_mm_movemask_ps(_mm_cmpgt_ps(a.m,nodeMax.m))*16)&0x77,
//		(_mm_movemask_ps(_mm_cmplt_ps(b.m,nodeMin.m))+_mm_movemask_ps(_mm_cmpgt_ps(b.m,nodeMax.m))*16)&0x77,
//		(_mm_movemask_ps(_mm_cmplt_ps(c.m,nodeMin.m))+_mm_movemask_ps(_mm_cmpgt_ps(c.m,nodeMax.m))*16)&0x77 };
//
//	if((out[0]==out[1]&&out[1]==out[2])&&out[0])
//		return 0;

	Vec3p ta=a*negMask,tb=b*negMask,tc=c*negMask;

	floatq yzyz[3];
	yzyz[0].m=_mm_shuffle(1*1+2*4+1*16+2*64,ta.m);
	yzyz[1].m=_mm_shuffle(1*1+2*4+1*16+2*64,tb.m);
	yzyz[2].m=_mm_shuffle(1*1+2*4+1*16+2*64,tc.m);
	floatq p[3]={pv[0]*floatq(ta.X())+ov[0],pv[0]*floatq(tb.X())+ov[0],pv[0]*floatq(tc.X())+ov[0]};
	int mask[3]={ForWhich(yzyz[0]<=p[0]),ForWhich(yzyz[1]<=p[1]),ForWhich(yzyz[2]<=p[2])};

	int mi=mask[0]&mask[1]&mask[2],ma=mask[0]|mask[1]|mask[2];
	bool lessMin=(mi&3),moreMax=(ma&4)==0||(ma&8)==0;
	lessMin=moreMax=0;

	if(!(lessMin||moreMax)) {
		floatq xyxy[3];
		xyxy[0].m=_mm_shuffle(0*1+1*4+0*16+1*64,ta.m);
		xyxy[1].m=_mm_shuffle(0*1+1*4+0*16+1*64,tb.m);
		xyxy[2].m=_mm_shuffle(0*1+1*4+0*16+1*64,tc.m);
		floatq p[3]={pv[2]*floatq(ta.Z())+ov[2],pv[2]*floatq(tb.Z())+ov[2],pv[2]*floatq(tc.Z())+ov[2]};
		int mask[3]={ForWhich(xyxy[0]<=p[0]),ForWhich(xyxy[1]<=p[1]),ForWhich(xyxy[2]<=p[2])};

		int mi=mask[0]&mask[1]&mask[2],ma=mask[0]|mask[1]|mask[2];
		bool lessMin=(mi&3),moreMax=(ma&12)^12;


		if(!(lessMin||moreMax)) {
			floatq xzxz[3];
			xzxz[0].m=_mm_shuffle(0*1+2*4+0*16+2*64,ta.m);
			xzxz[1].m=_mm_shuffle(0*1+2*4+0*16+2*64,tb.m);
			xzxz[2].m=_mm_shuffle(0*1+2*4+0*16+2*64,tc.m);
			floatq p[3]={pv[1]*floatq(ta.Y())+ov[1],pv[1]*floatq(tb.Y())+ov[1],pv[1]*floatq(tc.Y())+ov[1]};
			int mask[3]={ForWhich(xzxz[0]<=p[0]),ForWhich(xzxz[1]<=p[1]),ForWhich(xzxz[2]<=p[2])};
			int mi=mask[0]&mask[1]&mask[2],ma=mask[0]|mask[1]|mask[2];
			bool lessMin=(mi&3),moreMax=(ma&4)==0||(ma&8)==0;
			lessMin=moreMax=0;

			return !(lessMin||moreMax);
		}
	}

	return 0;
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
	INLINE bool BeamCollide(const floatq *pv,const floatq *ov,const Vec3p &negMask,
							const Vec3p &min,const Vec3p &max) const {
		switch(type) {
		case T_TRIANGLE:
			return tris[id].BeamCollide(pv,ov,negMask,min,max);
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
			out=Vec(tris[id].nrm);
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

