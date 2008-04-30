#ifndef RTRACER_CONTEXT_H
#define RTRACER_CONTEXT_H

#include "rtbase.h"
#include "ray_group.h"


struct TracingOptions {
	TracingOptions() { }
	TracingOptions(uint refl,bool rdt) :reflections(refl),rdtscShader(rdt) { }

	uint reflections;
	bool rdtscShader;
};

template <class Scene,class Rays,class Selector>
class TracingContext {
public:
	TracingContext(const Scene &scn,const Rays &tRays) :scene(scn),rays(tRays) {
		selector.SelectAll();
	}
	TracingContext(const Scene &scn,const Rays &tRays,const Selector &sel)
		:scene(scn),rays(tRays),selector(sel) {
	}

	Vec3q &RayDir(int n)				{ return rays.Dir(n); }
	const Vec3q &RayDir(int n) const	{ return rays.Dir(n); }

	Vec3q &RayOrigin(int n)				{ return rays.Origin(n); }
	const Vec3q &RayOrigin(int n) const	{ return rays.Origin(n); }

	Vec3q RayIDir(int n) const { return rays.IDir(n); }

	typedef Vec3q Vec;
	typedef typename Vec::TScalar real;
	typedef typename Vec::TBool boolean;
	typedef typename Scene::Object Object;
	typedef i32x4 integer;

	enum { size=Rays::size };

	TracingOptions options;
	const Scene &scene;

	Rays rays;
	Selector selector;

	Vec color[size],normal[size],position[size],light[size];
	Vec reflectionDir[size];

	real distance[size];
	integer objId[size];

	TreeStats stats;
	float density;
};

enum OutputType {
	otNormal,
	otPrimary,
	otShadow,
};

//
// Output classes hold pointers, so you can
// still modify the data when referencing to class
// with a const reference

template <OutputType type_,class real,class integer>
struct Output
{
	enum { objectIndexes=type_!=otShadow, type=type_ };

	Output(real *d,integer *i,TreeStats *st) :dist(d),object(i),stats(st),density(0) { }

	template <class Scene,class Group,class Selector>
	Output(TracingContext<Scene,Group,Selector> &c) :dist(c.distance),object(c.objId),stats(&c.stats),density(&c.density) { }
	Output(const Output &all,int n) :dist(all.dist+n),object(all.object+n*4),stats(all.stats),density(0) { }

	float *density;
	real *dist;
	integer *object;
	TreeStats *stats;
};


#endif
