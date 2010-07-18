#ifndef RTRACER_RAY_GROUP_H
#define RTRACER_RAY_GROUP_H

#include "rtbase.h"

extern f32x4b GetSSEMaskFromBits_array[16];

inline f32x4b GetSSEMaskFromBits(u8 bits) {
	/*
	union { u32 tmp[4]; __m128 out; };

	tmp[0]=bits&1?~0:0;
	tmp[1]=bits&2?~0:0;
	tmp[2]=bits&4?~0:0;
	tmp[3]=bits&8?~0:0;

	return out; */
	return GetSSEMaskFromBits_array[bits];
}

inline int CountMaskBits(u8 bits) {
	static const char count[16] = { 0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4 };
	return count[bits];
}

//TODO: zabezpieczyc operacje na 4 elementach na raz
class RaySelector {
public:
	RaySelector(char *mask, int size) :mask(mask), size(size), size4((size + 3) / 4) { }
	operator char*() { return mask; }
	operator const char*() const { return mask; }

	char operator[](int n) const { return mask[n]; }
	char &operator[](int n) { return mask[n]; }
	const f32x4b SSEMask(int n) const { return GetSSEMaskFromBits(mask[n]); }
	int Mask4(int n) const { return mask4[n]; }
	int &Mask4(int n) { return mask4[n]; }

	bool Any() const {
		for(int n = 0; n < size4; n++)
			if(mask4[n])
				return 1;
		return 0;
	}
	bool All() const {
		for(int n = 0; n < size4 - 1; n++)
			if(mask4[n] != 0x0f0f0f0f)
				return 0;

		int lastMask = mask4[size4 - 1] | (0x0f0f0f0f << (8 * (4 - (size4 * 4 - size))));
		return lastMask == 0x0f0f0f0f;
	}
	int Size() const { return size; }
	int Size4() const { return size4; }

	void Clear() {
		for(int n = 0; n < size4; n++)
			mask4[n] = 0;
	}
	void SelectAll() {
		for(int n = 0; n < size4; n++)
			mask4[n] = 0x0f0f0f0f;
	}

private:
	union {
		char *mask;
		int *mask4;
	};
	int size, size4;
};

template <bool sharedOrigin_, bool hasMask_>
class RayGroup
{
public:
	enum { sharedOrigin = sharedOrigin_, hasMask = hasMask_ };

	RayGroup(const Vec3q *origin, const Vec3q *dir, const Vec3q *iDir, int size)
		:size(size), origin(origin), dir(dir), iDir(iDir) { }
	RayGroup(const RayGroup &rhs, int offset, int newSize)
		:size(newSize), origin(rhs.origin + (sharedOrigin? 0 : offset)), dir(rhs.dir + offset),
		iDir(rhs.iDir + offset) { }
	RayGroup(const RayGroup<sharedOrigin, 1> &rhs, int offset, int newSize)
		:size(newSize), origin(rhs.origin + (sharedOrigin? 0 : offset)), dir(rhs.dir + offset),
		iDir(rhs.iDir + offset) { }

	const Vec3q &Dir(int n) const	{ return dir[n]; }
	const Vec3q &Origin(int n) const{ return origin[sharedOrigin? 0 : n]; }
	const Vec3q &IDir(int n) const	{ return iDir[n]; }
	char Mask(int n) const			{ return 15; }
	const f32x4b SSEMask(int n)const{ return _mm_set1_ps(UValue(~0).f); }

	const Vec3q *DirPtr() const		{ return dir; }
	const Vec3q *OriginPtr() const	{ return origin; }
	const Vec3q *IDirPtr() const	{ return iDir; }
	char *MaskPtr() const			{ return 0; }

	int Size() const { return size; }

	bool Any() const { return 1; }
	bool All() const { return 1; }

private:
	int size;
	const Vec3q *__restrict__ dir;
	const Vec3q *__restrict__ iDir;
	const Vec3q *__restrict__ origin;
};

template <bool sharedOrigin_>
class RayGroup<sharedOrigin_, true>
{
public:
	enum { sharedOrigin = sharedOrigin_, hasMask = true };

	RayGroup(const Vec3q *origin, const Vec3q *dir, const Vec3q *iDir, int size, char *mask)
		:size(size), origin(origin), dir(dir), iDir(iDir), mask(mask) { }
	RayGroup(const Vec3q *origin, const Vec3q *dir, const Vec3q *iDir, RaySelector &selector)
		:size(selector.Size()), origin(origin), dir(dir), iDir(iDir), mask(&selector[0]) { }
	RayGroup(const RayGroup &rhs, int offset, int newSize)
		:size(newSize), origin(rhs.origin + (sharedOrigin? 0 : offset)), dir(rhs.dir + offset),
		iDir(rhs.iDir + offset), mask(rhs.mask + offset) { }

	const Vec3q &Dir(int n) const	{ return dir[n]; }
	const Vec3q &Origin(int n) const{ return origin[sharedOrigin? 0 : n]; }
	const Vec3q &IDir(int n) const	{ return iDir[n]; }
	char Mask(int n) const			{ return mask[n]; }
	const f32x4b SSEMask(int n) const{ return GetSSEMaskFromBits(mask[n]); }

	const Vec3q *DirPtr() const		{ return dir; }
	const Vec3q *OriginPtr() const	{ return origin; }
	const Vec3q *IDirPtr() const	{ return iDir; }
	char *MaskPtr() const			{ return mask; }

	int Size() const { return size; }

	bool Any() const {
		for(int n = 0; n < size; n++)
			if(mask[n])
				return 1;
		return 0;
	}
	bool All() const {
		for(int n = 0; n < size; n++)
			if(mask[n] != 15)
				return 0;
		return 1;
	}


private:
	friend class RayGroup<sharedOrigin, false>;
	int size;
	const Vec3q *__restrict__ dir;
	const Vec3q *__restrict__ iDir;
	const Vec3q *__restrict__ origin;
	char *mask;
};

class Frustum
{
public:
	Frustum() { }
	template <bool shared, bool hasMask>
	explicit Frustum(const RayGroup<shared, hasMask> &gr) {
		int majorAxis = MaxAxis(ExtractN(gr.Dir(0), 0));
		static_assert(!hasMask, "masked groups not supported yet");

	/*	for(int axis = 0; axis < 3; axis++) {
			auto sign = floatq( ((&gr.Dir(0).x)[axis] < floatq(0.0f)).m );
			if(sign[0] != sign[1] || sign[0] != sign[2] || sign[0] != sign[3])
				continue;

			int q = 1;
			for(; q < size; q++)
				if(ForAny( floatq( ((&gr.Dir(q).x)[axis] < floatq(0.0f)).m ) != sign ))
					break;

			if(q == size) {
				majorAxis = axis;
				break;
			}
		} */

		int uAxis = (majorAxis + 1) % 3, vAxis = (majorAxis + 2) % 3;
		float maxUS, minUS, maxVS, minVS; {
			floatq maxUSq = (&gr.Dir(0).x)[uAxis] * (&gr.IDir(0).x)[majorAxis];
			floatq minUSq = maxUSq;

			for(int q = 1; q < gr.Size(); q++) {
				floatq slope = (&gr.Dir(q).x)[uAxis] * (&gr.IDir(q).x)[majorAxis];
				maxUSq = Max(maxUSq, slope);
				minUSq = Min(minUSq, slope);
			}
			
			floatq maxVSq = (&gr.Dir(0).x)[vAxis] * (&gr.IDir(0).x)[majorAxis];
			floatq minVSq = maxVSq;
			for(int q = 1; q < gr.Size(); q++) {
				floatq slope = (&gr.Dir(q).x)[vAxis] * (&gr.IDir(q).x)[majorAxis];
				maxVSq = Max(maxVSq, slope);
				minVSq = Min(minVSq, slope);
			}
			maxUS = Maximize(maxUSq); minUS = Minimize(minUSq);
			maxVS = Maximize(maxVSq); minVS = Minimize(minVSq);
		}
		
		bool flip = (&gr.Dir(0).x)[majorAxis][0] < 0.0f;

		Vec3f nrm[4];
		(&nrm[0].x)[uAxis] =  1.0f; (&nrm[0].x)[majorAxis] = -maxUS; (&nrm[0].x)[vAxis] = 0.0f;
		(&nrm[1].x)[uAxis] = -1.0f; (&nrm[1].x)[majorAxis] =  minUS; (&nrm[1].x)[vAxis] = 0.0f;
		(&nrm[2].x)[vAxis] =  1.0f; (&nrm[2].x)[majorAxis] = -maxVS; (&nrm[2].x)[uAxis] = 0.0f;
		(&nrm[3].x)[vAxis] = -1.0f; (&nrm[3].x)[majorAxis] =  minVS; (&nrm[3].x)[uAxis] = 0.0f;

		for(int n = 0; n < 4; n++)
			nrm[n] *= RSqrt(nrm[n] | nrm[n]);
		if(!flip) for(int n = 0; n < 4; n++)
			nrm[n] = -nrm[n];

		if(shared) {
			Vec3f origin = ExtractN(gr.Origin(0), 0);
			for(int p = 0; p < 4; p++)
				planes[p] = Plane(origin, nrm[p]);
		}
		else {
			Vec3f omin, omax;
			ComputeMinMax(&gr.Origin(0), gr.Size(), &omin, &omax);
			exit(0);
			for(int p = 0; p < 4; p++) {

			}
		}
	}

	Plane planes[4];
};

//TODO specjalizacja dla roznych originow?
class CornerRays
{
public:
	CornerRays() { }

	template <bool shared, bool hasMask>
	explicit CornerRays(const RayGroup<shared, hasMask> &gr) {
		if(hasMask) return; //TODO
		int size = gr.Size();

		Vec3f td[4];
		td[0] = ExtractN(gr. Dir( size == 4? 0 : size == 16?  0 : size == 32?  0 : size == 64?  0 : size == 128? 0  : 0  ), 0);
		td[1] = ExtractN(gr. Dir( size == 4? 1 : size == 16?  5 : size == 32? 19 : size == 64? 21 : size == 128? 85 : 85 ), 1);
		td[2] = ExtractN(gr. Dir( size == 4? 2 : size == 16? 10 : size == 32? 10 : size == 64? 42 : size == 128? 42 : 170), 2);
		td[3] = ExtractN(gr. Dir( size == 4? 3 : size == 16? 15 : size == 32? 31 : size == 64? 63 : size == 128? 127: 255), 3);
		Convert(td, dir);
		td[0] = ExtractN(gr.IDir( size == 4? 0 : size == 16?  0 : size == 32?  0 : size == 64?  0 : size == 128? 0  : 0  ), 0);
		td[1] = ExtractN(gr.IDir( size == 4? 1 : size == 16?  5 : size == 32? 19 : size == 64? 21 : size == 128? 85 : 85 ), 1);
		td[2] = ExtractN(gr.IDir( size == 4? 2 : size == 16? 10 : size == 32? 10 : size == 64? 42 : size == 128? 42 : 170), 2);
		td[3] = ExtractN(gr.IDir( size == 4? 3 : size == 16? 15 : size == 32? 31 : size == 64? 63 : size == 128? 127: 255), 3);
		Convert(td, idir);
		origin = ExtractN(gr.Origin(0), 0);
	}
	explicit CornerRays(const Frustum &frustum) {
		Vec3f rays[4];
		Intersect(frustum.planes[0], frustum.planes[2], rays[0], origin);
		Intersect(frustum.planes[0], frustum.planes[3], rays[1], origin);
		Intersect(frustum.planes[1], frustum.planes[2], rays[2], origin);
		Intersect(frustum.planes[1], frustum.planes[3], rays[3], origin);
	}

	// Order:
	// 0  -  1
	// |     |
	// 2  -  3
	Vec3q dir, idir;
	Vec3f origin;
};

class RayInterval {
public:
	RayInterval() { }
	RayInterval(const RayGroup<1, 0> &rays, floatq *__restrict__ distanceMask) {
		int size = rays.Size();

		ComputeMinMax(rays.DirPtr(), distanceMask, size, &minDir, &maxDir);
		ComputeMinMax(rays.IDirPtr(), distanceMask, size, &minIDir, &maxIDir);
		minOrigin = maxOrigin = ExtractN(rays.Origin(0), 0);
		
		ix = floatq(minIDir.x, maxIDir.x, minIDir.x, maxIDir.x);
		iy = floatq(minIDir.y, maxIDir.y, minIDir.y, maxIDir.y);
		iz = floatq(minIDir.z, maxIDir.z, minIDir.z, maxIDir.z);
	}
	
	template <bool shared, bool hasMask>
	explicit RayInterval(const RayGroup<shared, hasMask> &rays) {
		int size = rays.Size();

		if(hasMask) {
			ComputeMinMax(rays.DirPtr(), rays.MaskPtr(), size, &minDir, &maxDir);
			ComputeMinMax(rays.IDirPtr(), rays.MaskPtr(), size, &minIDir, &maxIDir);
		}
		else {
			ComputeMinMax(rays.DirPtr(), size, &minDir, &maxDir);
			ComputeMinMax(rays.IDirPtr(), size, &minIDir, &maxIDir);
		}

		if(shared)
			minOrigin = maxOrigin = ExtractN(rays.Origin(0), 0);
		else {
			if(hasMask)
				ComputeMinMax(rays.OriginPtr(), rays.MaskPtr(), size, &minOrigin, &maxOrigin);
			else
				ComputeMinMax(rays.OriginPtr(), size, &minOrigin, &maxOrigin);
		}
		
		ix = floatq(minIDir.x, maxIDir.x, minIDir.x, maxIDir.x);
		iy = floatq(minIDir.y, maxIDir.y, minIDir.y, maxIDir.y);
		iz = floatq(minIDir.z, maxIDir.z, minIDir.z, maxIDir.z);
	}
	
	floatq ix, iy, iz;
	Vec3f minIDir, maxIDir, minDir, maxDir;
	Vec3f minOrigin, maxOrigin;
};

struct ShadowCache {
	ShadowCache() { Clear(); }
	void Clear() { lastTri=~0; }
	int Size() const { return lastTri==~0?0:1; }
	void Insert(int idx) { lastTri=idx; }
	int operator[](int n) const { return lastTri; }

private:
	int lastTri;
};

template <bool sharedOrigin, bool hasMask>
struct Context {
	Context(const RayGroup<sharedOrigin, hasMask> &tRays, floatq *distance, i32x4 *object, i32x4 *element,
			Vec2q *barycentric, TreeStats *stats = 0)
		:rays(tRays), distance(distance), object(object), element(element), stats(stats), barycentric(barycentric) { }

	const Vec3q &Dir(int q) const { return rays.Dir(q); }
	const Vec3q &IDir(int q) const { return rays.IDir(q); }
	const Vec3q &Origin(int q) const { return rays.Origin(q); }
	char Mask(int q) const { return rays.Mask(q); }
	const f32x4b SSEMask(int q) const { return rays.SSEMask(q); }
	char *MaskPtr() { return rays.MaskPtr(); }

	f32x4 &Distance(int q) { return distance[q]; }
	i32x4 &Object(int q) { return object[q]; }
	i32x4 &Element(int q) { return element[q]; }
	int Size() const { return rays.Size(); }

	void UpdateStats(const TreeStats &st) { if(stats) *stats += st; }

	bool All() const { return rays.All(); }
	bool Any() const { return rays.Any(); }

	RayGroup<sharedOrigin, hasMask> rays;
	floatq * __restrict__ distance;
	i32x4  * __restrict__ object;
	i32x4  * __restrict__ element;
	Vec2q * __restrict__ barycentric;
	TreeStats *stats;
};

// Rays with distance < 0 are masked
struct ShadowContext {
	ShadowContext(const RayGroup<1, 0> &tRays, floatq *distance, TreeStats *stats = 0)
		:rays(tRays), distance(distance), stats(stats) { }

	const Vec3q &Dir(int q) const { return rays.Dir(q); }
	const Vec3q &IDir(int q) const { return rays.IDir(q); }
	const Vec3q &Origin(int q) const { return rays.Origin(q); }

	f32x4 &Distance(int q) { return distance[q]; }
	int Size() const { return rays.Size(); }

	void UpdateStats(const TreeStats &st) { if(stats) *stats += st; }

	bool All() const { return rays.All(); }
	bool Any() const { return rays.Any(); }

	RayGroup<1, 0> rays;
	floatq * __restrict__ distance;
	ShadowCache shadowCache;
	TreeStats *stats;
};


/*
template <int flags_>
struct FContext {
	enum { flags=flags_, sharedOrigin=flags&isct::fShOrig };

	FContext(const Context<1,flags> &c,int o) {
		const Vec3q &tOrig=c.Origin(0);
		const Vec3q &tDir=c.Dir(0);
		const Vec3q &tIDir=c.IDir(0);
		origin=Vec3f(tOrig.x[o],tOrig.y[o],tOrig.z[o]);
		dir=Vec3f(tDir.x[o],tDir.y[o],tDir.z[o]);
		iDir=Vec3f(tIDir.x[o],tIDir.y[o],tIDir.z[o]);
		
		distance = &c.distance[0][o];
		object   = &c.object  [0][o];
		element  = &c.element [0][o];
		if(flags&isct::fShadow) shadowCache=c.shadowCache;
		stats=c.stats;
	}

	const Vec3f &Dir(int) const { return dir; }
	const Vec3f &IDir(int) const { return iDir; }
	const Vec3f &Origin(int) const { return origin; }

	float &Distance(int) { return distance[0]; }
	int   &Object  (int) { return object  [0]; }
	int   &Element (int) { return element [0]; }
	
	void UpdateStats(const TreeStats &st) { if(stats) *stats+=st; }

	Vec3f origin,dir,iDir;
	float *__restrict__ distance;
	int   * __restrict__ object;
	int   * __restrict__ element;
	ShadowCache shadowCache;
	TreeStats *stats;
};
*/

#endif

