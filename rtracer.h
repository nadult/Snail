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

INLINE void Print(const char *name,__m128 val)
{
	float t[4];
	Convert(floatq(val),t);
	printf("%s: %f %f %f %f\n",name,t[0],t[1],t[2],t[3]);
}

INLINE bool Triangle::BeamCollide(const floatq *pv,const floatq *ov,const Vec3p &negMask,
									const Vec3p &minOR,const Vec3p &maxOR) const
{
	Vec3p ta=a*negMask,tb=b*negMask,tc=c*negMask,tnrm=nrm*negMask;
	bool lessMin,moreMax;

	/*
	Vec4p tv[3]={pv[0].m,pv[1].m,pv[2].m};
	Vec3p tdir[3]={
		Vec3p(2,tv[0].X(),tv[0].Y())+Vec3p(2,tv[0].Z(),tv[0].W()),
		Vec3p(tv[1].X(),2,tv[1].Y())+Vec3p(tv[1].Z(),2,tv[1].W()),
		Vec3p(tv[2].X(),tv[2].Y(),2)+Vec3p(tv[2].Z(),tv[2].W(),2) };

	Vec3p dir=(tdir[0]+tdir[1]+tdir[2])*floatp(1.0f/12.0f);
	Vec3p orig=(minOR+maxOR)*floatp(0.5); */

	Vec3p dir=maxOR,orig=minOR*negMask;

	floatp vn=nrm|dir;
	floatp v0=(orig|nrm)+(a|nrm);
	floatp t=v0/vn;
	Vec3p col=orig+dir*t;
//	if(ForAny(t<Const<floatq,0>::Value())) return 0;

	Print("a",a.m); Print("b",b.m); Print("c",c.m); Print("dir",dir.m);
	Print("col",col.m); Print("nrm",nrm.m); Print("orig",orig.m); Print("t",t.m);

	Vec3p e[3]={(b-a),(c-b),(a-c)};
	Print("e0",e[0].m); Print("e1",e[1].m); Print("e2",e[2].m);

	Vec3p en[3]={e[0]^nrm,e[1]^nrm,e[2]^nrm};
	en[0]*=RSqrt(en[0]|en[0]);
	en[1]*=RSqrt(en[1]|en[1]);
	en[2]*=RSqrt(en[2]|en[2]);

	Print("en0",en[0].m); Print("en1",en[1].m); Print("en2",en[2].m);

	floatq dist[3]={(col-a)|en[0],(col-b)|en[1],(col-c)|en[2]};
	Print("d0",dist[0].m); Print("d1",dist[1].m); Print("d2",dist[2].m);

	throw 0;
				
	if(ForAny(dist[0]>=Const<floatp,-100,1>::Value()&&
			dist[1]>=Const<floatp,-100,1>::Value()&&
			dist[2]>=Const<floatp,-100,1>::Value()))
		return 1;
	return 0;

//	Vec3p pa=(A|dir)*dir; floatq dista=(A-pa)|(A-pa);
//	Vec3p pb=(B|dir)*dir; floatq distb=(B-pb)|(B-pb);
//	Vec3p pc=(C|dir)*dir; floatq distc=(C-pc)|(C-pc);

//	if(ForAny(dir.Z()<Const<floatq,0>::Value())) {
//	if(ForAll(Min(dista,Min(distb,distc))>=Const<floatp,4>::Value())) {
//		Print("dista",dista.m);
//		Print("dir",dir.m);
//		Print("orig",orig.m);
//		Print("A",A.m);
//		throw 0;
//		return 0;
//	}

	return 1;

	// Test on plane X -------------------------
	{ floatq yzyz[3];
	yzyz[0].m=_mm_shuffle(1*1+2*4+1*16+2*64,ta.m);
	yzyz[1].m=_mm_shuffle(1*1+2*4+1*16+2*64,tb.m);
	yzyz[2].m=_mm_shuffle(1*1+2*4+1*16+2*64,tc.m);
	floatq p[3]={pv[0]*floatq(ta.X())+ov[0],pv[0]*floatq(tb.X())+ov[0],pv[0]*floatq(tc.X())+ov[0]};
	int mask[3]={ForWhich(yzyz[0]<=p[0]),ForWhich(yzyz[1]<=p[1]),ForWhich(yzyz[2]<=p[2])};

	int mi=mask[0]&mask[1]&mask[2],ma=mask[0]|mask[1]|mask[2];
	lessMin=(mi&3),moreMax=(ma&12)^12; }

	if(lessMin||moreMax) return 0;
	
	{
		float tv[4]; Convert(pv[0],tv);
		Vec3p nrmy[2]={Vec3p(tv[0],1,0),Vec3p(tv[2],1,0)};
		Vec3p nrmz[2]={Vec3p(tv[1],0,1),Vec3p(tv[3],0,1)};
		Vec3p yp[2]={Vec3p(maxOR.X(),minOR.Y(),0),Vec3p(minOR.X(),maxOR.Y(),0)};
		Vec3p zp[2]={Vec3p(maxOR.X(),0,minOR.Z()),Vec3p(minOR.X(),0,maxOR.Z())};
		floatp yd[2]={nrmy[0]|yp[0],nrmy[1]|yp[1]};
		floatp zd[2]={nrmz[0]|zp[0],nrmz[1]|zp[1]};
			
		bool out[4]={
			ForAny((ta|nrmy[0])<yd[0]&&(tb|nrmy[0])<yd[0]&&(tc|nrmy[0])<yd[0]),
			ForAny((ta|nrmy[1])<yd[1]&&(tb|nrmy[1])<yd[1]&&(tc|nrmy[1])<yd[1]),
			ForAny((ta|nrmz[0])<zd[0]&&(tb|nrmz[0])<zd[0]&&(tc|nrmz[0])<zd[0]),
			ForAny((ta|nrmz[1])<zd[1]&&(tb|nrmz[1])<zd[1]&&(tc|nrmz[1])<zd[1]) };

		if(out[0]||out[1]||out[2]||out[3]) return 0;
	}

	// Test on plane Y -------------------
	{ floatq xzxz[3];
	xzxz[0].m=_mm_shuffle(0*1+2*4+0*16+2*64,ta.m);
	xzxz[1].m=_mm_shuffle(0*1+2*4+0*16+2*64,tb.m);
	xzxz[2].m=_mm_shuffle(0*1+2*4+0*16+2*64,tc.m);
	floatq p[3]={pv[1]*floatq(ta.Y())+ov[1],pv[1]*floatq(tb.Y())+ov[1],pv[1]*floatq(tc.Y())+ov[1]};
	int mask[3]={ForWhich(xzxz[0]<=p[0]),ForWhich(xzxz[1]<=p[1]),ForWhich(xzxz[2]<=p[2])};
	int mi=mask[0]&mask[1]&mask[2],ma=mask[0]|mask[1]|mask[2];
	lessMin=(mi&3),moreMax=(ma&12)^12; }

	if(lessMin||moreMax) return 0;
	
	{
		float tv[4]; Convert(pv[2],tv);
		Vec3p nrmx[2]={Vec3p(1,tv[0],0),Vec3p(1,tv[2],0)};
		Vec3p nrmz[2]={Vec3p(0,tv[1],1),Vec3p(0,tv[3],1)};
		Vec3p xp[2]={Vec3p(minOR.X(),maxOR.Y(),0),Vec3p(maxOR.X(),minOR.Y(),0)};
		Vec3p zp[2]={Vec3p(0,maxOR.Y(),minOR.Z()),Vec3p(0,minOR.Y(),maxOR.Z())};
		floatp xd[2]={nrmx[0]|xp[0],nrmx[1]|xp[1]};
		floatp zd[2]={nrmz[0]|zp[0],nrmz[1]|zp[1]};
			
		bool out[4]={
			ForAny((ta|nrmx[0])<xd[0]&&(tb|nrmx[0])<xd[0]&&(tc|nrmx[0])<xd[0]),
			ForAny((ta|nrmx[1])<xd[1]&&(tb|nrmx[1])<xd[1]&&(tc|nrmx[1])<xd[1]),
			ForAny((ta|nrmz[0])<zd[0]&&(tb|nrmz[0])<zd[0]&&(tc|nrmz[0])<zd[0]),
			ForAny((ta|nrmz[1])<zd[1]&&(tb|nrmz[1])<zd[1]&&(tc|nrmz[1])<zd[1]) };

		if(out[0]||out[1]||out[2]||out[3]) return 0;
	}

	// Test on plane Z ----------------------
	{ floatq xyxy[3];
	xyxy[0].m=_mm_shuffle(0*1+1*4+0*16+1*64,ta.m);
	xyxy[1].m=_mm_shuffle(0*1+1*4+0*16+1*64,tb.m);
	xyxy[2].m=_mm_shuffle(0*1+1*4+0*16+1*64,tc.m);
	floatq p[3]={pv[2]*floatq(ta.Z())+ov[2],pv[2]*floatq(tb.Z())+ov[2],pv[2]*floatq(tc.Z())+ov[2]};
	int mask[3]={ForWhich(xyxy[0]<=p[0]),ForWhich(xyxy[1]<=p[1]),ForWhich(xyxy[2]<=p[2])};
	int mi=mask[0]&mask[1]&mask[2],ma=mask[0]|mask[1]|mask[2];
	lessMin=(mi&3),moreMax=(ma&12)^12; }

	if(lessMin||moreMax) return 0;

	{
		float tv[4]; Convert(pv[2],tv);
		Vec3p nrmx[2]={Vec3p(1,0,tv[0]),Vec3p(1,0,tv[2])};
		Vec3p nrmy[2]={Vec3p(0,1,tv[1]),Vec3p(0,1,tv[3])};
		Vec3p xp[2]={Vec3p(minOR.X(),0,maxOR.Z()),Vec3p(maxOR.X(),0,minOR.Z())};
		Vec3p yp[2]={Vec3p(0,minOR.Y(),maxOR.Z()),Vec3p(0,maxOR.Y(),minOR.Z())};
		floatp xd[2]={nrmx[0]|xp[0],nrmx[1]|xp[1]};
		floatp yd[2]={nrmy[0]|yp[0],nrmy[1]|yp[1]};
			
		bool out[4]={
			ForAny((ta|nrmx[0])<xd[0]&&(tb|nrmx[0])<xd[0]&&(tc|nrmx[0])<xd[0]),
			ForAny((ta|nrmx[1])<xd[1]&&(tb|nrmx[1])<xd[1]&&(tc|nrmx[1])<xd[1]),
			ForAny((ta|nrmy[0])<yd[0]&&(tb|nrmy[0])<yd[0]&&(tc|nrmy[0])<yd[0]),
			ForAny((ta|nrmy[1])<yd[1]&&(tb|nrmy[1])<yd[1]&&(tc|nrmy[1])<yd[1]) };

		if(out[0]||out[1]||out[2]||out[3]) return 0;
	}

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

