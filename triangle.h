#ifndef RTRACER_TRIANGLE_H
#define RTRACER_TRIANGLE_H

#include "rtbase.h"
#include "ray_group.h"

class Cache;

class TriAccel
{
public:
	enum {
		isComplex=0, isctFlags=isct::fDistance
	};

	TriAccel() { }
	TriAccel(const Vec3f &p0, const Vec3f &p1, const Vec3f &p2) {
		Vec3f a = p0, b = p1, c = p2;
		nrm = (b - a) ^ (c - a);

		float inv_nw;
		bool  sign = 0;
		{
			float a[3] = { Abs(nrm.x), Abs(nrm.y), Abs(nrm.z) };

			iw = a[0] > a[1] ? 0 : 1;
			iw = a[2] > a[iw] ? 2 : iw;

			if((&nrm.x)[iw] < 0.0f) {
				Swap(b, c);
				nrm  = -nrm;
				sign = 1;
			}

			inv_nw = 1.0f / (&nrm.x)[iw];

			if(iw == 0) {
				iu = 1;
				iv = 2;
			}
			else if(iw == 1) {
				iu = 0;
				iv = 2;
			}
			else {
				iu = 0;
				iv = 1;
			}

			nu = (&nrm.x)[iu] * inv_nw;
			nv = (&nrm.x)[iv] * inv_nw;
		}
		{
			pu = (&a.x)[iu];
			pv = (&a.x)[iv];
			np = nu * pu + nv * pv + (&p0.x)[iw];
		}
		{
			Vec3f edge0 = b - a, edge1 = c - a;
			float sign  = iw == 1 ? -1.0f : 1.0f;
			float mul   = sign * inv_nw;

			e0u = (&edge0.x)[iu] * mul;
			e0v = (&edge0.x)[iv] * mul;
			e1u = (&edge1.x)[iu] * mul;
			e1v = (&edge1.x)[iv] * mul;
		}
		flags  = 0;
		flags |= sign ? 1 : 0;
		flags |= Abs(nu) == 0.0f && Abs(nv) == 0.0f ? 2 : 0;

		if(sign) nrm = -nrm;
		nrm *= RSqrt(nrm | nrm);
	}

	const Vec3f &Nrm() const {
		return nrm;
	}

/*	template <int tflags>
	void Collide(FContext <tflags> &c, int idx) const {
		bool sign = flags & 1;

		float dirW  = (&c.dir.x)[iw], dirU = (&c.dir.x)[iu], dirV = (&c.dir.x)[iv];
		float origW = (&c.origin.x)[iw], origU = (&c.origin.x)[iu], origV = (&c.origin.x)[iv];

		float det  = dirU * nu + dirV * nv + dirW;
		float dett = np - (origU * nu + origV * nv + origW);
		float dist = dett / det;

		float Du   = dirU * dett - det * (pu - origU);
		float Dv   = dirV * dett - det * (pv - origV);
		float detu = Du * e1v - Dv * e1u;
		float detv = Dv * e0u - Du * e0v;

		float tmp  = detu + detv;
		bool  mask = sign ? detu <= 0.0f && detv <= 0.0f && det <= tmp : detu >= 0.0f && detv >= 0.0f && det >= tmp;

		mask = mask && dist > 0.0f && dist < c.distance[0];

		c.distance[0] = Condition(mask, dist, c.distance[0]);
		if(!(tflags & isct::fShadow)) c.object  [0] = Condition(mask, idx, c.object  [0]);
	}

	template <int w, int tflags, int size>
	int Collide_(Context <size, tflags> &c, int idx) const {
		floatq tdett, ppu, ppv;

		if(tflags & isct::fShOrig) {
			Vec3q orig = c.Origin(0);
			tdett = np - ((&orig.x)[(int)iu][0] * nu + (&orig.x)[(int)iv][0] * nv + (&orig.x)[(int)w][0]);
			ppu   = floatq(pu) - (&orig.x)[iu];
			ppv   = floatq(pv) - (&orig.x)[iv];
		}

		bool sign = flags & 1;
		bool full = tflags & isct::fShadow ? 1 : 0;

		if(EXPECT_NOT_TAKEN(flags & 2)) for(int q = 0; q < size; q++) {
				const Vec3q dir    = c.Dir(q);
				const Vec3q origin = c.Origin(q);

				floatq dirW = w == 0 ? dir.x : w == 1 ? dir.y : dir.z, origW = w == 0 ? origin.x : w == 1 ? origin.y : origin.z;
				floatq dirU = w == 0 ? dir.y : w == 1 ? dir.x : dir.x, origU = w == 0 ? origin.y : w == 1 ? origin.x : origin.x;
				floatq dirV = w == 0 ? dir.z : w == 1 ? dir.z : dir.y, origV = w == 0 ? origin.z : w == 1 ? origin.z : origin.y;

				floatq det  = dirW;
				floatq dett = tflags & isct::fShOrig ? tdett : floatq(np) - origW;
				floatq dist = dett / det;

				floatq Du   = dirU * dett - det * (tflags & isct::fShOrig ? ppu : floatq(pu) - origU);
				floatq Dv   = dirV * dett - det * (tflags & isct::fShOrig ? ppv : floatq(pv) - origV);
				floatq detu = Du * e1v - Dv * e1u;
				floatq detv = Dv * e0u - Du * e0v;

				floatq tmp  = detu + detv;
				f32x4b mask = sign ? detu <= 0.0f && detv <= 0.0f && det <= tmp : detu >= 0.0f && detv >= 0.0f && det >= tmp;
				mask = mask && dist > 0.0f && dist < c.distance[q];

				if(tflags & isct::fShadow) full &= ForAll(mask);

				c.distance[q] = Condition(mask, dist, c.distance[q]);
				if(!(tflags & isct::fShadow)) c.object[q] = Condition(i32x4b(mask), i32x4(idx), c.object[q]);
			}
		else for(int q = 0; q < size; q++) {
				const Vec3q dir    = c.Dir(q);
				const Vec3q origin = c.Origin(q);

				floatq dirW = w == 0 ? dir.x : w == 1 ? dir.y : dir.z, origW = w == 0 ? origin.x : w == 1 ? origin.y : origin.z;
				floatq dirU = w == 0 ? dir.y : w == 1 ? dir.x : dir.x, origU = w == 0 ? origin.y : w == 1 ? origin.x : origin.x;
				floatq dirV = w == 0 ? dir.z : w == 1 ? dir.z : dir.y, origV = w == 0 ? origin.z : w == 1 ? origin.z : origin.y;

				floatq det  = dirU * nu + dirV * nv + dirW;
				floatq dett = tflags & isct::fShOrig ? tdett : floatq(np) - (origU * nu + origV * nv + origW);
				floatq dist = dett / det;

				floatq Du   = dirU * dett - det * (tflags & isct::fShOrig ? ppu : floatq(pu) - origU);
				floatq Dv   = dirV * dett - det * (tflags & isct::fShOrig ? ppv : floatq(pv) - origV);
				floatq detu = Du * e1v - Dv * e1u;
				floatq detv = Dv * e0u - Du * e0v;

				floatq tmp  = detu + detv;
				f32x4b mask = sign ? detu <= 0.0f && detv <= 0.0f && det <= tmp : detu >= 0.0f && detv >= 0.0f && det >= tmp;
				mask = mask && dist > 0.0f && dist < c.distance[q];

				if(tflags & isct::fShadow) full &= ForAll(mask);

				c.distance[q] = Condition(mask, dist, c.distance[q]);
				if(!(tflags & isct::fShadow)) c.object[q] = Condition(i32x4b(mask), i32x4(idx), c.object[q]);
			}

		return full;
	}

	template <int tflags, int size>
	int Collide(Context <size, tflags> &c, int idx) const {
		return iw == 0 ? Collide_ <0>(c, idx) :
			   iw == 1 ? Collide_ <1>(c, idx) :
			   Collide_ <2>(c, idx);
	} */

private:
	float nu, nv;     // normal
	float np, pu, pv; // vertex
	float e0u, e0v;   // edge 0
	float e1u, e1v;   // edge 1
	char  iw, iu, iv; // indices
	char  flags;      // 1: sign

	Vec3f nrm;
	int   temp[3];
};

static_assert(sizeof(TriAccel) == 64, "TriAccel size should be 64");

class Triangle
{
public:
	enum {
		isctFlags = isct::fDistance,
		isComplex = 0 // doesn't contain any hierarchy of objects
	};

	Triangle(const Vec3f ta, const Vec3f b, const Vec3f c) {
		a = ta;
		ba = b - a;
		ca = c - a;
		ComputeData();
	}

	Triangle() { }

	bool Test() const {
		bool nan = isnan(a.x) || isnan(a.y) || isnan(a.z);

		nan = nan || isnan(ba.x) || isnan(ba.y) || isnan(ba.z);
		nan = nan || isnan(ca.x) || isnan(ca.y) || isnan(ca.z);
		nan = nan || isnan(plane.x) || isnan(plane.y) || isnan(plane.z);

		return !nan;
	}

	const Vec3f P1() const { return a; }
	const Vec3f P2() const { return ba + a; }
	const Vec3f P3() const { return ca + a; }

	const Vec3f Edge1() const { return ba; }
	const Vec3f Edge2() const { return ca - ba; }
	const Vec3f Edge3() const { return -ca; }

	const Vec3f Edge1Normal() const {
		Vec3f tmp = Nrm() ^ Edge1();
		return tmp * RSqrt(tmp | tmp);
	}

	const Vec3f Edge2Normal() const {
		Vec3f tmp = Nrm() ^ Edge2();
		return tmp * RSqrt(tmp | tmp);
	}

	const Vec3f Edge3Normal() const {
		Vec3f tmp = Nrm() ^ Edge3();
		return tmp * RSqrt(tmp | tmp);
	}

	const Vec3f Nrm() const { return Vec3f(plane); }
	const Vec3f Nrm(int) const { return Nrm(); }

	const Vec3f BoundMin() const {
		return VMin(P1(), VMin(P2(), P3()));
	}
	const Vec3f BoundMax() const {
		return VMax(P1(), VMax(P2(), P3()));
	}
	const BBox GetBBox() const {
		return BBox(BoundMin(), BoundMax());
	}

	template <class Vec>
	const Vec Normal(const Vec &) const {
		return Vec(Nrm());
	}

	template <int flags, class VecO, class Vec>
	const Isct <typename Vec::TScalar, 1, isct::fDistance | flags>
	Collide(VecO rOrig, Vec rDir) const __attribute__((noinline));

	template <int flags, class VecO, class Vec>
	const Isct <typename Vec::TScalar, 1, isct::fDistance | flags>
	Collide(VecO rOrig, Vec rDir, float maxDist) const {
		return Collide <flags>(rOrig, rDir);
	}

	template <bool sharedOrigin, bool hasMask>
	void Collide(Context<sharedOrigin, hasMask> &c, int idx, int firstActive, int lastActive) const;
	
	template <bool sharedOrigin, bool hasMask>
	int CollideShadow(Context<sharedOrigin, hasMask> &c, int firstActive, int lastActive) const;

	template <class Vec0, class Vec, class Real>
	const Vec3 <typename Vec::TScalar> Barycentric(Vec0 rOrig, Vec rDir, Real dist, int) const;

	bool TestInterval(Vec3f origin, Vec3f minDir, Vec3f maxDir) const {
		Vec3f nrm = Nrm();
		Vec3f nrm1 = nrm * minDir, nrm2 = nrm * maxDir;

		float det = Max(nrm1.x, nrm2.x) + Max(nrm1.y, nrm2.y) + Max(nrm1.z, nrm2.z);
		if(det < 0.0f)
			return 0;

		Vec3f tvec = origin - a;
		Vec3f c1 = ba ^ tvec, c2 = tvec ^ ca;
		Vec3f c1a = minDir * c1, c1b = maxDir * c1;
		Vec3f c2a = minDir * c2, c2b = maxDir * c2;

		float u[2] = {
			Min(c1a.x, c1b.x) + Min(c1a.y, c1b.y) + Min(c1a.z, c1b.z),
			Max(c1a.x, c1b.x) + Max(c1a.y, c1b.y) + Max(c1a.z, c1b.z) };
		float v[2] = {
			Min(c2a.x, c2b.x) + Min(c2a.y, c2b.y) + Min(c2a.z, c2b.z),
			Max(c2a.x, c2b.x) + Max(c2a.y, c2b.y) + Max(c2a.z, c2b.z) };
		
		return Min(u[1], v[1]) >= 0.0f && u[0] + v[0] <= det * t0;
	}


	bool TestCornerRays(const CornerRays&) const;
	bool TestInterval(const RayInterval&) const;
	bool TestFrustum(const Frustum&) const;

	void ComputeData() {
		Vec3f nrm      = (ba) ^ (ca);
		float e1ce2Len = Length(nrm);

		nrm  /= e1ce2Len;
		t0 = e1ce2Len;
		it0 = 1.0f / t0;
		plane = Vec4f(nrm.x, nrm.y, nrm.z, nrm | a);
	}

	Vec3f a, ba, ca;
	float t0, it0; int temp[1];
	Vec4f plane;
};

SERIALIZE_AS_POD(Triangle)

template <int flags, class VecO, class Vec>
const Isct <typename Vec::TScalar, 1, isct::fDistance | flags> Triangle::Collide(VecO rOrig, Vec rDir) const {
	typedef typename Vec::TScalar    real;
	typedef typename Vec::TBool      Bool;

	Isct <typename Vec::TScalar, 1, isct::fDistance | flags> out;

	Vec3f ba  = P2() - P1();
	Vec3f ca  = P3() - P1();
	Vec3f nrm = ba ^ ca;
	float t0  = Length(nrm);
	nrm /= t0;

	real det  = rDir | nrm;
	VecO tvec = rOrig - VecO(a);
	real u    = rDir | (VecO(ba) ^ tvec);
	real v    = rDir | (tvec ^ VecO(ca));
	Bool test = Min(u, v) >= 0.0f && u + v <= det * real(t0);

//	if (ForAny(test)) {
	real dist = -(tvec | nrm) / det;
	out.Distance() = Condition(test, dist, real(1.0f / 0.0f));
//	}

	return out;
}

template <class VecO, class Vec, class Real>
const Vec3 <typename Vec::TScalar> Triangle::Barycentric(VecO rOrig, Vec rDir, Real dist, int) const {
	typedef typename Vec::TBool    Bool;
	Vec3 <typename Vec::TScalar> out;

	typename Vec::TScalar det = (rDir | Nrm()) * t0;
	VecO tvec = rOrig - VecO(a);
	typename Vec::TScalar idet = Inv(det);
	out.z = (rDir | (VecO(ba) ^ tvec)) * idet;
	out.y = (rDir | (tvec ^ VecO(ca))) * idet;
	out.x = (typename Vec::TScalar)(1.0f) - out.y - out.z;

	return out;
}

typedef vector <Triangle, AlignedAllocator <Triangle> > TriVector;

class ShTriangle {
public:
	Vec2f uv[3];
	Vec3f nrm[3];
	int   matId;

	enum {
		fFlatNormals = 1,
	};

	int   flags;
	u32   temp[2];

//	Vec3f tangent,binormal;

	ShTriangle() { }
	ShTriangle(const Vec2f &uv1, const Vec2f &uv2, const Vec2f &uv3,
			   const Vec3f &nrm1, const Vec3f &nrm2, const Vec3f &nrm3,
			   int tMatId, bool flatNrm = 0) {
		uv[0]  = uv1;
		uv[1]  = uv2;
		uv[2]  = uv3;
		nrm[0] = nrm1;
		nrm[1] = nrm2;
		nrm[2] = nrm3;
		flags  = flatNrm ? fFlatNormals : 0;

		matId = tMatId;

		uv[1] -= uv[0];
		uv[2] -= uv[0];

		nrm[1] -= nrm[0];
		nrm[2] -= nrm[0];

		/*
		 * Vec3p side0=pos[0]-pos[1];
		 * Vec3p side1=pos[2]-pos[0];
		 *
		 * float deltaV0=uv[0].y-uv[1].y;
		 * float deltaV1=uv[2].y-uv[0].y;
		 * tangent=side0*deltaV1-side1*deltaV0;
		 * tangent*=RSqrt(tangent|tangent);
		 *
		 * float deltaU0 = uv[0].x-uv[1].x;
		 * float deltaU1 = uv[2].x-uv[0].x;
		 * binormal=side0*deltaU1-side1*deltaU0;
		 * binormal*=RSqrt(binormal|binormal);
		 *
		 * ///Now, take the cross product of the tangents to get a vector which
		 * ///should point in the same direction as our normal calculated above.
		 * ///If it points in the opposite direction (the dot product between the normals is less than zero),
		 * ///then we need to reverse the s and t tangents.
		 * ///This is because the triangle has been mirrored when going from tangent space to object space.
		 * ///reverse tangents if necessary
		 * Vec3p tangentCross=tangent^binormal;
		 * if ((tangentCross|normal)<0.0f) {
		 *  tangent=-tangent;
		 *  binormal=-binormal;
		 * } */
	}

	static Vec3f Rot(const Vec3f *rot, const Vec3f vec) {
		return Vec3f(
				vec.x * rot[0].x + vec.y * rot[0].y + vec.z * rot[0].z,
				vec.x * rot[1].x + vec.y * rot[1].y + vec.z * rot[1].z,
				vec.x * rot[2].x + vec.y * rot[2].y + vec.z * rot[2].z);
	}

	void Transform(const Vec3f *rot, const Vec3f translation) {
		nrm[1] += nrm[0]; nrm[2] += nrm[0];
		nrm[0] = Rot(rot, nrm[0]);
		nrm[1] = Rot(rot, nrm[1]);
		nrm[2] = Rot(rot, nrm[2]);
		nrm[1] -= nrm[0]; nrm[2] -= nrm[0];
	}

	bool FlatNormals() const {
		return flags & fFlatNormals;
	}
};

typedef vector <ShTriangle, AlignedAllocator <ShTriangle> >    ShTriVector;

class BaseScene;

class TriangleVector {
public:
	typedef TriAccel      CElement;
	typedef ShTriangle    SElement;

	void Serialize(Serializer &sr) {
		sr &verts &tris &triAccels;
	}

	const CElement &GetCElement(int elem) const {
		return triAccels[elem];
	}
	const CElement &operator[](int elem) const {
		return GetCElement(elem);
	}

	const SElement GetSElement(int elem, int) const {
		const TriIdx &idx = tris[elem];
		const Vert &  v1  = verts[idx.v1], &v2 = verts[idx.v2], &v3 = verts[idx.v3];

		return ShTriangle(v1.uv, v2.uv, v3.uv, v1.nrm, v2.nrm, v3.nrm,
						  idx.mat & 0x7ffffff, idx.mat & 0x80000000 ? 1 : 0);
	}

	const Vec3f BoundMin(int n) const {
		const TriIdx &idx = tris[n];

		return VMin(verts[idx.v1].pos, VMin(verts[idx.v2].pos, verts[idx.v3].pos));
	}

	const Vec3f BoundMax(int n) const {
		const TriIdx &idx = tris[n];

		return VMax(verts[idx.v1].pos, VMax(verts[idx.v2].pos, verts[idx.v3].pos));
	}

	const BBox GetBBox(int n) const {
		const TriIdx &idx = tris[n];
		Vec3f         min = VMin(verts[idx.v1].pos, VMin(verts[idx.v2].pos, verts[idx.v3].pos));
		Vec3f         max = VMax(verts[idx.v1].pos, VMax(verts[idx.v2].pos, verts[idx.v3].pos));

		return BBox(min, max);
	}

	const Triangle ToTriangle(int n) const {
		const TriIdx &idx = tris[n];

		return Triangle(verts[idx.v1].pos, verts[idx.v2].pos, verts[idx.v3].pos);
	}

	size_t size() const {
		return tris.size();
	}
	size_t MemSize() const {
		return verts.size() * (sizeof(Vert)) + tris.size() * sizeof(TriIdx);
	}

//private:
	struct Vert {
		Vec3f pos, nrm;
		Vec2f uv;
	};
	vector <Vert> verts;

	struct TriIdx {
		u32 v1, v2, v3;
		u32 mat;
	};
	vector <TriIdx> tris;
	vector <TriAccel, AlignedAllocator <TriAccel> > triAccels;

	friend class BaseScene;
};

class CompactTris {
public:
	vector<Vec3f> verts;
	vector<Vec3f> normals;

	void Serialize(Serializer &sr) {
		sr &verts & normals & tris;
	}

	typedef Triangle      CElement;
	typedef ShTriangle    SElement;

	const CElement GetCElement(int elem) const {
		auto tri = tris[elem];
		return Triangle(verts[tri.v1], verts[tri.v2], verts[tri.v3]);
	}
	const CElement operator[](int elem) const {
		return GetCElement(elem);
	}

	const SElement GetSElement(int elem, int) const {
		const TriIdx &idx = tris[elem];
		const Vec2f uv(0.0f, 0.0f);

		return ShTriangle(uv, uv, uv, normals[idx.v1], normals[idx.v2],
				normals[idx.v3], idx.mat & 0x7ffffff, idx.mat & 0x80000000 ? 1 : 0);
	}

	const Vec3f BoundMin(int n) const {
		const TriIdx &idx = tris[n];

		return VMin(verts[idx.v1], VMin(verts[idx.v2], verts[idx.v3]));
	}

	const Vec3f BoundMax(int n) const {
		const TriIdx &idx = tris[n];

		return VMax(verts[idx.v1], VMax(verts[idx.v2], verts[idx.v3]));
	}

	const BBox GetBBox(int n) const {
		const TriIdx &idx = tris[n];
		Vec3f min = VMin(verts[idx.v1], VMin(verts[idx.v2], verts[idx.v3]));
		Vec3f max = VMax(verts[idx.v1], VMax(verts[idx.v2], verts[idx.v3]));

		return BBox(min, max);
	}

	size_t size() const {
		return tris.size();
	}
	size_t MemSize() const {
		return verts.size() * (sizeof(Vec3f) * 2) + tris.size() * sizeof(TriIdx);
	}

	struct TriIdx { u32 v1, v2, v3, mat; };
	vector<TriIdx> tris;
};

SERIALIZE_AS_POD(TriangleVector::TriIdx)
SERIALIZE_AS_POD(TriangleVector::Vert)
SERIALIZE_AS_POD(TriAccel)

SERIALIZE_AS_POD(CompactTris::TriIdx)



// After changing the element, you have to set the id because it will be trashed
template <class Tri, int tsize = 64>
class TriCache {
public:
	enum { size = tsize };

	TriCache() {
		for(int n = 0; n < size; n++) {
			data[n].temp[0] = ~0;
			data[n].temp[1] = ~0;
		}
	}

	uint Hash(u32 objId, u32 elemId) const {
		return (objId + elemId) & (size - 1);
	}

	Tri &operator[](uint idx) {
		return data[idx];
	}
	const Tri &operator[](uint idx) const {
		return data[idx];
	}

	bool SameId(uint idx, u32 objId, u32 elemId) const {
		const Tri &d = data[idx];

		return d.temp[0] == objId && d.temp[1] == elemId;
	}

	void SetId(uint idx, u32 objId, u32 elemId) {
		Tri &d = data[idx];

		d.temp[0] = objId;
		d.temp[1] = elemId;
	}

private:
	Tri data[size];
};

typedef TriCache<Triangle, 64> TriangleCache;

#endif
