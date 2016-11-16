#ifndef RTRACER_TRIANGLE_H
#define RTRACER_TRIANGLE_H

#include "rtbase.h"
#include "ray_group.h"

class Cache;

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
	
	bool Collide(ShadowContext &c, int firstActive, int lastActive) const;

	template <class Vec0, class Vec, class Real>
	const Vec3 <typename Vec::TScalar> Barycentric(Vec0 rOrig, Vec rDir, Real dist, int) const;

	bool TestInterval(Vec3f origin, Vec3f minDir, Vec3f maxDir) const {
		Vec3f nrm = Nrm();
		Vec3f nrm1 = nrm * minDir, nrm2 = nrm * maxDir;

		float det = Max(nrm1.x, nrm2.x) + Max(nrm1.y, nrm2.y) + Max(nrm1.z, nrm2.z);
	//	if(det < 0.0f)
	//		return 0;

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

SERIALIZE_AS_POD(::Triangle)

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

class ShTriangle {
public:
	Vec2f uv[3];
	Vec3f nrm[3];

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

		matId = tMatId | (flatNrm? 0x80000000 : 0);

		uv[1] -= uv[0];
		uv[2] -= uv[0];

		nrm[1] -= nrm[0];
		nrm[2] -= nrm[0];
	}

	static const Vec3f Rot(const Vec3f *rot, const Vec3f vec) {
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
		return matId & 0x80000000;
	}
	int MatId() const {
		return matId & 0x7fffffff;
	}
	
protected:
	int matId;
};

SERIALIZE_AS_POD(ShTriangle)

class BaseScene;

class CompactTris {
public:
	vector<Vec3f> verts;

	struct ShData {
		ShData() { }
		ShData(const Vec3f normal, const Vec2f uv) :normal(normal), uv(uv) { }

		Vec3f normal;
		Vec2f uv;
	};

	vector<ShData> shData;

	void save(Stream &sr) const { sr << verts << shData << tris; }
	void load(Stream &sr) { sr >> verts >> shData >> tris; }

	typedef Triangle      CElement;
	typedef ShTriangle    SElement;

	const CElement GetCElement(int elem) const {
		const TriIdx tri = tris[elem];
		return Triangle(verts[tri.v1], verts[tri.v2], verts[tri.v3]);
	}
	const CElement operator[](int elem) const {
		return GetCElement(elem);
	}

	const SElement GetSElement(int elem, int) const {
		const TriIdx &idx = tris[elem];
		const ShData &sh1 = shData[idx.v1], &sh2 = shData[idx.v2], &sh3 = shData[idx.v3];

		return ShTriangle(sh1.uv, sh2.uv, sh3.uv, sh1.normal, sh2.normal, sh3.normal,
				idx.mat & 0x7ffffff, idx.mat & 0x80000000 ? 1 : 0);
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

SERIALIZE_AS_POD(CompactTris::TriIdx)
SERIALIZE_AS_POD(CompactTris::ShData)


// After changing the element, you have to set the id because it will be trashed
template <class Tri, int tsize = 64>
class TriCache {
public:
	enum { size = tsize };

	TriCache() {
		for(int n = 0; n < size; n++)
			ids[n] = Id(~0, ~0);
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
		return ids[idx] == Id(objId, elemId);
	}

	void SetId(uint idx, u32 objId, u32 elemId) {
		ids[idx] = Id(objId, elemId);
	}

private:
	struct Id {
		Id() { }
		Id(int obj, int elem)
			:id((((unsigned long long)obj) << 32) + elem) { }

		bool operator==(const Id rhs)
			{ return id == rhs.id; }

		unsigned long long id;
	};

	Tri data[size];
	Id ids[size];
};

typedef vector<Triangle> TriVector;
typedef vector<ShTriangle> ShTriVector;
typedef std::vector<Triangle, AlignedAllocator<Triangle, 256> > ATriVector;
typedef std::vector<ShTriangle, AlignedAllocator<ShTriangle, 256> > AShTriVector;

#endif
