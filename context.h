#ifndef RTRACER_CONTEXT_H
#define RTRACER_CONTEXT_H

#include "rtbase.h"
#include "ray_group.h"
#include "tree_stats.h"

enum ShadingMode {
	smFlat,
	smGouraud,
};


struct TracingOptions {
	TracingOptions() { }
	TracingOptions(uint refl,ShadingMode sm,bool rdt) :reflections(refl),shadingMode(sm),rdtscShader(rdt) { }

	uint reflections;
	ShadingMode shadingMode;
	bool rdtscShader;
};

struct ShadowCache {
	enum { size=4 };
	ShadowCache() { for(int n=0;n<size;n++) lastTri[n]=-1; }
	int &operator[](int n) { return lastTri[n]; }
	int operator[](int n) const { return lastTri[n]; }

private:
	int lastTri[size];
};

template <class Rays,class Selector>
class TracingContext {
public:
	TracingContext(const Rays &tRays) :rays(tRays) {
		selector.SelectAll();
	}
	TracingContext(const Rays &tRays,const Selector &sel)
		:rays(tRays),selector(sel) {
	}

	Vec3q &RayDir(int n)				{ return rays.Dir(n); }
	const Vec3q &RayDir(int n) const	{ return rays.Dir(n); }

	Vec3q &RayOrigin(int n)				{ return rays.Origin(n); }
	const Vec3q &RayOrigin(int n) const	{ return rays.Origin(n); }

	Vec3q RayIDir(int n) const { return rays.IDir(n); }

	typedef Vec3q Vec;
	typedef typename Vec::TScalar real;
	typedef typename Vec::TBool boolean;
	typedef i32x4 integer;

	enum { size=Rays::size };

	TracingOptions options;

	Rays rays;
	Selector selector;

	Vec color[size],normal[size],position[size],light[size];
	Vec reflectionDir[size];

	real distance[size];
	integer objId[size],elementId[size];
	
	ShadowCache shadowCache;
};

#endif
