#ifndef RTRACER_TRIANGLE_H
#define RTRACER_TRIANGLE_H

#include "rtbase.h"
#include "ray_group.h"

class Cache;

class TriAccel
{
public:
	enum { isComplex=0, isctFlags=isct::fDistance };

	TriAccel() { }
	TriAccel(const Vec3f &p0,const Vec3f &p1,const Vec3f &p2) {
		Vec3f a=p0,b=p1,c=p2;
		nrm=(b-a)^(c-a);

		float inv_nw;
		bool sign=0;
		{
			float a[3]={Abs(nrm.x),Abs(nrm.y),Abs(nrm.z)};

			iw=a[0]>a[1]?0:1;
			iw=a[2]>a[iw]?2:iw;

			if((&nrm.x)[iw]<0.0f) { Swap(b,c); nrm=-nrm; sign=1; }

			inv_nw=1.0f/(&nrm.x)[iw];

			if(iw==0) { iu=1; iv=2; }
			else if(iw==1) { iu=0; iv=2; }
			else { iu=0; iv=1; }

			nu=(&nrm.x)[iu]*inv_nw;
			nv=(&nrm.x)[iv]*inv_nw;
		}
		{
			pu=(&a.x)[iu];
			pv=(&a.x)[iv];
			np=nu*pu+nv*pv+(&p0.x)[iw];
		}
		{
			Vec3f edge0=b-a,edge1=c-a;
			float sign=iw==1?-1.0f:1.0f;
			float mul=sign*inv_nw;

			e0u=(&edge0.x)[iu]*mul;
			e0v=(&edge0.x)[iv]*mul;
			e1u=(&edge1.x)[iu]*mul;
			e1v=(&edge1.x)[iv]*mul;
		}
		flags=0;
		flags|=sign?1:0;
		flags|=Abs(nu)==0.0f&&Abs(nv)==0.0f?2:0;

		if(sign) nrm=-nrm;
		nrm*=RSqrt(nrm|nrm);
	}

	const Vec3f &Nrm() const { return nrm; }

	template <int tflags>
	void Collide(FContext<tflags> &c,int idx) const {
		bool sign=flags&1;

		float dirW=(&c.dir.x)[iw],dirU=(&c.dir.x)[iu],dirV=(&c.dir.x)[iv];
		float origW=(&c.origin.x)[iw],origU=(&c.origin.x)[iu],origV=(&c.origin.x)[iv];

		float det=dirU*nu+dirV*nv+dirW;
		float dett=np-(origU*nu+origV*nv+origW);
		float dist=dett/det;

		float Du=dirU*dett-det*(pu-origU);
		float Dv=dirV*dett-det*(pv-origV);
		float detu=Du*e1v-Dv*e1u;
		float detv=Dv*e0u-Du*e0v;

		float tmp=detu+detv;
		bool mask=sign?detu<=0.0f&&detv<=0.0f&&det<=tmp:detu>=0.0f&&detv>=0.0f&&det>=tmp;
		mask=mask&&dist>0.0f&&dist<c.distance[0];
	
		c.distance[0]=Condition(mask,dist,c.distance[0]);
		if(!(tflags&isct::fShadow)) c.object  [0]=Condition(mask,idx ,c.object  [0]);
	}

	template <int w,int tflags,int size>
	int Collide_(Context<size,tflags> &c,int idx) const {
		floatq tdett,ppu,ppv;
		if(tflags&isct::fShOrig) {
			Vec3q orig=c.Origin(0);
			tdett=np-((&orig.x)[iu][0]*nu+(&orig.x)[iv][0]*nv+(&orig.x)[w][0]);
			ppu=floatq(pu)-(&orig.x)[iu];
			ppv=floatq(pv)-(&orig.x)[iv];
		}

		bool sign=flags&1;
		bool full=tflags&isct::fShadow?1:0;

		if(EXPECT_NOT_TAKEN(flags&2)) for(int q=0;q<size;q++) {
			const Vec3q dir=c.Dir(q);
			const Vec3q origin=c.Origin(q);

			floatq dirW=w==0?dir.x:w==1?dir.y:dir.z,origW=w==0?origin.x:w==1?origin.y:origin.z;
			floatq dirU=w==0?dir.y:w==1?dir.x:dir.x,origU=w==0?origin.y:w==1?origin.x:origin.x;
			floatq dirV=w==0?dir.z:w==1?dir.z:dir.y,origV=w==0?origin.z:w==1?origin.z:origin.y;

			floatq det=dirW;
			floatq dett=tflags&isct::fShOrig?tdett:floatq(np)-origW;
			floatq dist=dett/det;

			floatq Du=dirU*dett-det*(tflags&isct::fShOrig?ppu:floatq(pu)-origU);
			floatq Dv=dirV*dett-det*(tflags&isct::fShOrig?ppv:floatq(pv)-origV);
			floatq detu=Du*e1v-Dv*e1u;
			floatq detv=Dv*e0u-Du*e0v;

			floatq tmp=detu+detv;
			f32x4b mask=sign?detu<=0.0f&&detv<=0.0f&&det<=tmp:detu>=0.0f&&detv>=0.0f&&det>=tmp;
			mask=mask&&dist>0.0f&&dist<c.distance[q];

			if(tflags&isct::fShadow) full&=ForAll(mask);

			c.distance[q]=Condition(mask,dist,c.distance[q]);
			if(!(tflags&isct::fShadow)) c.object[q]=Condition(i32x4b(mask),i32x4(idx),c.object[q]);
		}
		else for(int q=0;q<size;q++) {
			const Vec3q dir=c.Dir(q);
			const Vec3q origin=c.Origin(q);

			floatq dirW=w==0?dir.x:w==1?dir.y:dir.z,origW=w==0?origin.x:w==1?origin.y:origin.z;
			floatq dirU=w==0?dir.y:w==1?dir.x:dir.x,origU=w==0?origin.y:w==1?origin.x:origin.x;
			floatq dirV=w==0?dir.z:w==1?dir.z:dir.y,origV=w==0?origin.z:w==1?origin.z:origin.y;

			floatq det=dirU*nu+dirV*nv+dirW;
			floatq dett=tflags&isct::fShOrig?tdett:floatq(np)-(origU*nu+origV*nv+origW);
			floatq dist=dett/det;

			floatq Du=dirU*dett-det*(tflags&isct::fShOrig?ppu:floatq(pu)-origU);
			floatq Dv=dirV*dett-det*(tflags&isct::fShOrig?ppv:floatq(pv)-origV);
			floatq detu=Du*e1v-Dv*e1u;
			floatq detv=Dv*e0u-Du*e0v;

			floatq tmp=detu+detv;
			f32x4b mask=sign?detu<=0.0f&&detv<=0.0f&&det<=tmp:detu>=0.0f&&detv>=0.0f&&det>=tmp;
			mask=mask&&dist>0.0f&&dist<c.distance[q];

			if(tflags&isct::fShadow) full&=ForAll(mask);

			c.distance[q]=Condition(mask,dist,c.distance[q]);
			if(!(tflags&isct::fShadow)) c.object[q]=Condition(i32x4b(mask),i32x4(idx),c.object[q]);
		}

		return full;
	}
	
	template <int tflags,int size>
	int Collide(Context<size,tflags> &c,int idx) const {
		return iw==0?Collide_<0>(c,idx):iw==1?Collide_<1>(c,idx):Collide_<2>(c,idx);
	}

private:
	float nu,nv;   // normal
	float np,pu,pv;// vertex
	float e0u,e0v; // edge 0
	float e1u,e1v; // edge 1
	char iw,iu,iv; // indices
	char flags;	   // 1: sign

	Vec3f nrm;
	int temp[3];
};

static_assert(sizeof(TriAccel)==64,"bah");

class Triangle
{
public:
	enum { isctFlags=isct::fDistance };
	enum { isComplex=0 }; // doesn't contain any hierarchy of objects

	Triangle(const Vec3f &ta,const Vec3f &tb,const Vec3f &tc) {
		Vec3p b,c;
		Convert(ta,a); Convert(tb,b); Convert(tc,c);
		ba=b-a; ca=c-a;
		ComputeData();
	}
	Triangle() { }

	bool Test() const {
		bool nan=isnan(a.x)||isnan(a.y)||isnan(a.z);
		nan=nan||isnan(ba.x)||isnan(ba.y)||isnan(ba.z);
		nan=nan||isnan(ca.x)||isnan(ca.y)||isnan(ca.z);
		nan=nan||isnan(plane.x)||isnan(plane.y)||isnan(plane.z);

		return !nan;
	}

	INLINE Vec3p P1() const { return a; }
	INLINE Vec3p P2() const { return ba+a; }
	INLINE Vec3p P3() const { return ca+a; }

	INLINE Vec3p Edge1() const { return ba; }
	INLINE Vec3p Edge2() const { return ca-ba; }
	INLINE Vec3p Edge3() const { return -ca; }

	inline Vec3p Edge1Normal() const { Vec3p tmp=Nrm()^Edge1(); return tmp*RSqrt(tmp|tmp); }
	inline Vec3p Edge2Normal() const { Vec3p tmp=Nrm()^Edge2(); return tmp*RSqrt(tmp|tmp); }
	inline Vec3p Edge3Normal() const { Vec3p tmp=Nrm()^Edge3(); return tmp*RSqrt(tmp|tmp); }

	inline Vec3p Nrm() const { return Vec3p(plane); }
	INLINE Vec3p Nrm(int) const { return Nrm(); }

	inline Vec3p BoundMin() const { return VMin(P1(),VMin(P2(),P3())); }
	inline Vec3p BoundMax() const { return VMax(P1(),VMax(P2(),P3())); }

	template <class Vec>
	inline Vec Normal(const Vec&) const { return Vec(Nrm()); }

	template <int flags,class VecO,class Vec>
	Isct<typename Vec::TScalar,1,isct::fDistance|flags>
		Collide(const VecO &rOrig,const Vec &rDir) const NOINLINE;

	template <int flags,class VecO,class Vec>
	INLINE Isct<typename Vec::TScalar,1,isct::fDistance|flags>
		Collide(const VecO &rOrig,const Vec &rDir,float maxDist) const
		{ return Collide<flags>(rOrig,rDir); }

	template <int flags,int packetSize>
	Isct<f32x4,packetSize,isct::fDistance|flags>
		Collide(const RayGroup<packetSize,flags> &rays) const NOINLINE;

	template <class Vec0,class Vec,class Real>
	Vec3<typename Vec::TScalar> Barycentric(const Vec0 &rOrig,const Vec &rDir,const Real &dist,int) const;

	int PrimaryBeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL) const;
	int BeamCollide(const Vec3p &orig,const Vec3p &dir,float epsL) const;

private:
	void SetFlag1(uint value) { a.t0=UValue(value).f; }
	uint GetFlag1() const { return UValue(a.t0).i; }

public:
	void SetFlag2(uint value) { ba.t0=UValue(value).f; }
	uint GetFlag2() const { return UValue(ba.t0).i; }

private:
	void ComputeData() {
		Vec3p nrm=(ba)^(ca);
		float e1ce2Len=Length(nrm);
		nrm/=e1ce2Len;
		ca.t0=e1ce2Len;
		plane=Vec4p(nrm.x,nrm.y,nrm.z,nrm|a);
	}

private:
	Vec3p a,ba,ca;
	Vec4p plane;
};

template <int flags,class VecO,class Vec>
Isct<typename Vec::TScalar,1,isct::fDistance|flags> Triangle::Collide(const VecO &rOrig,const Vec &rDir) const {
	typedef typename Vec::TScalar real;
	typedef typename Vec::TBool Bool;

	Isct<typename Vec::TScalar,1,isct::fDistance|flags> out;

	Vec3f ba=P2()-P1();
	Vec3f ca=P3()-P1();
	Vec3f nrm=ba^ca;
	float t0=Length(nrm);
	nrm/=t0;

	real det = rDir|nrm;
	VecO tvec = rOrig-VecO(a);
	real u = rDir|(VecO(ba)^tvec);
	real v = rDir|(tvec^VecO(ca));
	Bool test=Min(u,v)>=0.0f&&u+v<=det*real(t0);

//	if (ForAny(test)) {
		real dist=-(tvec|nrm)/det;
		out.Distance()=Condition(test,dist,real(1.0f/0.0f));
//	}

	return out;
}

template <int flags,int packetSize>
Isct<f32x4,packetSize,isct::fDistance|flags> Triangle::Collide(const RayGroup<packetSize,flags> &rays) const {
	Isct<f32x4,packetSize,isct::fDistance|flags> out;

	Vec3p nrm=Nrm();
	Vec3q ta(a);

	Vec3q sharedTVec;
	if(flags&isct::fShOrig) sharedTVec=rays.Origin(0)-ta;
	f32x4 infinity=1.0f/0.0f;

	for(int q=0;q<packetSize;q++) {
		floatq det=rays.Dir(q)|nrm;
		Vec3q tvec=flags&isct::fShOrig?sharedTVec:rays.Origin(q)-ta;

		floatq u=rays.Dir(q)|(Vec3q(ba)^tvec);
		floatq v=rays.Dir(q)|(tvec^Vec3q(ca));
		f32x4b test=Min(u,v)>=0.0f&&u+v<=det*floatq(ca.t0);

	//	if(ForAny(test)) {
			floatq dist=-(tvec|nrm)/det;
			out.Distance(q)=Condition(test,dist,infinity);
	//	}
	}

	return out;
}

template <class VecO,class Vec,class Real>
Vec3<typename Vec::TScalar> Triangle::Barycentric(const VecO &rOrig,const Vec &rDir,const Real &dist,int) const {
	typedef typename Vec::TBool Bool;
	Vec3<typename Vec::TScalar> out;

	typename Vec::TScalar det = (rDir|Nrm())*ca.t0;
	VecO tvec = rOrig-VecO(a);
	typename Vec::TScalar idet=Inv(det);
	out.z = (rDir|(VecO(ba)^tvec))*idet;
	out.y = (rDir|(tvec^VecO(ca)))*idet;
	out.x=(typename Vec::TScalar)(1.0f)-out.y-out.z;

	return out;
}

typedef vector<Triangle,AlignedAllocator<Triangle> > TriVector;

class ShTriangle {
public:
	Vec2f uv[3];
	Vec3f nrm[3];
	int matId;

	enum {
		fFlatNormals = 1,
	};

	float t0;
	Vec3f ba,ca,a;
	Vec3f normal;
	int flags;
	u32 temp[2];

//	Vec3f tangent,binormal;

	ShTriangle() { }
	ShTriangle(const Vec3f &p1,const Vec3p &p2,const Vec3f &p3,const Vec2f &uv1,const Vec2f &uv2,const Vec2f &uv3,
				const Vec3f &nrm1,const Vec3f &nrm2,const Vec3f &nrm3,int tMatId,bool flatNrm=0) {
		Vec3f pos[3];

		pos[0]=p1; pos[1]=p2; pos[2]=p3;
		uv[0]=uv1; uv[1]=uv2; uv[2]=uv3;
		nrm[0]=nrm1; nrm[1]=nrm2; nrm[2]=nrm3;
		flags=flatNrm?fFlatNormals:0;
	
		normal=(pos[1]-pos[0])^(pos[2]-pos[0]);
		t0=Length(normal);
		normal*=1.0f/t0;

		a=pos[0];
		ba=pos[1]-a;
		ca=pos[2]-a;
		matId=tMatId;

		uv[1]-=uv[0];
		uv[2]-=uv[0];

		nrm[1]-=nrm[0];
		nrm[2]-=nrm[0];

		/*
		Vec3p side0=pos[0]-pos[1];
		Vec3p side1=pos[2]-pos[0];

		float deltaV0=uv[0].y-uv[1].y;
		float deltaV1=uv[2].y-uv[0].y;
		tangent=side0*deltaV1-side1*deltaV0;
		tangent*=RSqrt(tangent|tangent);

		float deltaU0 = uv[0].x-uv[1].x;
		float deltaU1 = uv[2].x-uv[0].x;
		binormal=side0*deltaU1-side1*deltaU0;
		binormal*=RSqrt(binormal|binormal);

		///Now, take the cross product of the tangents to get a vector which 
		///should point in the same direction as our normal calculated above. 
		///If it points in the opposite direction (the dot product between the normals is less than zero), 
		///then we need to reverse the s and t tangents. 
		///This is because the triangle has been mirrored when going from tangent space to object space.
		///reverse tangents if necessary
		Vec3p tangentCross=tangent^binormal;
		if ((tangentCross|normal)<0.0f) {
			tangent=-tangent;
			binormal=-binormal;
		} */
	}
	void operator*=(const Matrix<Vec4f> &trans) {
		nrm[0]=trans&nrm[0];
		nrm[1]=trans&nrm[1];
		nrm[2]=trans&nrm[2];
		normal=trans&normal;
		Vec3f c=ca+a,b=ba+a;
		a=trans*a;
		b=trans*b;
		c=trans*c;
		ba=b-a; ca=c-a;
	}
	bool FlatNormals() const { return flags&fFlatNormals; }


	template <class Vec0,class Vec>
	Vec2<typename Vec::TScalar> Barycentric(const Vec0 &origin,const Vec &dir) const {
		typedef typename Vec::TScalar Real;
		Vec2<Real> out;

		Real det = (dir|normal)*Real(t0);
		Real idet=Inv(det);
		Vec0 tvec=origin-Vec0(a);
		out.x = (dir|(tvec^Vec0(ca)))*idet;
		out.y = (dir|(Vec0(ba)^tvec))*idet;

		return out;
	}
};

static_assert(sizeof(ShTriangle)==128,"blah");
typedef vector<ShTriangle,AlignedAllocator<ShTriangle> > ShTriVector;

class BaseScene;

class TriangleVector {
public:
	typedef TriAccel CElement;
	typedef ShTriangle SElement;

	void Serialize(Serializer &sr) { sr&verts&tris&triAccels; }

	INLINE const CElement &GetCElement(int elem) const { return triAccels[elem]; }
	INLINE const CElement &operator[](int elem) const { return GetCElement(elem); }

	INLINE const SElement GetSElement(int elem,int) const {
		const TriIdx &idx=tris[elem];
		const Vert &v1=verts[idx.v1],&v2=verts[idx.v2],&v3=verts[idx.v3];
		return ShTriangle(	v1.pos,v2.pos,v3.pos,v1.uv,v2.uv,v3.uv,v1.nrm,v2.nrm,v3.nrm,
							idx.mat&0x7ffffff,idx.mat&0x80000000?1:0);
	}
	Vec3f BoundMin(int n) const {
		const TriIdx &idx=tris[n];
		return VMin(verts[idx.v1].pos,VMin(verts[idx.v2].pos,verts[idx.v3].pos));
	}
	Vec3f BoundMax(int n) const {
		const TriIdx &idx=tris[n];
		return VMax(verts[idx.v1].pos,VMax(verts[idx.v2].pos,verts[idx.v3].pos));
	}

	BBox GetBBox(int n) const {
		const TriIdx &idx=tris[n];
		Vec3f min=VMin(verts[idx.v1].pos,VMin(verts[idx.v2].pos,verts[idx.v3].pos));
		Vec3f max=VMax(verts[idx.v1].pos,VMax(verts[idx.v2].pos,verts[idx.v3].pos));
		return BBox(min,max);
	}
	Triangle ToTriangle(int n) const {
		const TriIdx &idx=tris[n];
		return Triangle(verts[idx.v1].pos,verts[idx.v2].pos,verts[idx.v3].pos);
	}

	size_t size() const { return tris.size(); }
	size_t mem_size() const { return verts.size()*(sizeof(Vert))+tris.size()*sizeof(TriIdx); }

//private:
	struct Vert { Vec3f pos,nrm; Vec2f uv; };
	vector<Vert> verts;

	struct TriIdx { u32 v1,v2,v3; u32 mat; };
	vector<TriIdx> tris;
	vector<TriAccel,AlignedAllocator<TriAccel> > triAccels;

	friend class BaseScene;
};

namespace baselib {
	template<> struct SerializeAsPOD<TriangleVector::TriIdx> { enum { value=1 }; };
	template<> struct SerializeAsPOD<TriangleVector::Vert> { enum { value=1 }; };
	template<> struct SerializeAsPOD<TriAccel> { enum { value=1 }; };
}

	// After changing the element, you have to set the id because it will be trashed
	class ShTriCache {
	public:
		enum { size=64 };

		ShTriCache() {
			for(int n=0;n<size;n++) {
				data[n].temp[0]=~0;
				data[n].temp[1]=~0;
			}
		}

		INLINE uint Hash(u32 objId,u32 elemId) const { return (objId+elemId)&(size-1); }

		INLINE ShTriangle &operator[](uint idx) { return data[idx]; }
		INLINE const ShTriangle &operator[](uint idx) const { return data[idx]; }

		INLINE bool SameId(uint idx,u32 objId,u32 elemId) const {
			const ShTriangle &d=data[idx];
			return d.temp[0]==objId&&d.temp[1]==elemId;
		}
		INLINE void SetId(uint idx,u32 objId,u32 elemId) {
			ShTriangle &d=data[idx];
			d.temp[0]=objId;
			d.temp[1]=elemId;
		}

	private:
		ShTriangle data[size];
	};

#endif

