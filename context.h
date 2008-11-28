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

#endif
