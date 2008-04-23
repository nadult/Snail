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

template <class Scene,class Group,class Vec,class integer>
class TracingContext {
public:
	TracingContext(const Scene &scn,RayStore<Group::size,Group::singleOrigin> &rStore) :scene(scn),rayGroup(rStore) {
		selector.SelectAll();
	}
	TracingContext(const Scene &scn,RayStore<Group::size,Group::singleOrigin> &rStore,const RaySelector<Group::size> &sel)
		:scene(scn),rayGroup(rStore),selector(sel) {
		}

	typedef typename Vec::TScalar real;
	typedef typename Vec::TBool boolean;
	typedef typename Scene::Object Object;
	enum { size=Group::size };

	TracingOptions options;
	const Scene &scene;

	Group rayGroup;
	RaySelector<size> selector;

	Vec color[size],normal[size],position[size],light[size];
	Vec reflectionDir[size];

	real distance[size];
	integer objId[size];

	TreeStats stats;
};

//
// Output classes hold pointers, so you can
// still modify the data when referencing to class
// with a const reference

template <class real,class integer>
struct NormalOutput
{
	enum { objectIndexes=1, shadow=0 };

	NormalOutput(real *d,integer *i,TreeStats *st) :dist(d),object(i),stats(st) { }

	template <class Scene,class Group>
	NormalOutput(TracingContext<Scene,Group,Vec3<real>,integer> &c) :dist(c.distance),object(c.objId),stats(&c.stats) { }
	NormalOutput(const NormalOutput &all,int n) :dist(all.dist+n),object(all.object+n*4),stats(all.stats) { }

	real *dist;
	integer *object;
	TreeStats *stats;
};

template <class real,class integer>
struct ShadowOutput
{
	enum { objectIndexes=0, shadow=1 };
	
	ShadowOutput(real *d,TreeStats *st) :dist(d),object(0),stats(st) { }

	template <class Scene,class Group>
	ShadowOutput(TracingContext<Scene,Group,Vec3<real>,integer> &c) :dist(c.distance),object(0),stats(&c.stats) { }
	ShadowOutput(const ShadowOutput &all,int n) :dist(all.dist+n),object(0),stats(all.stats) { }

	real *dist;
	integer *object; // dummy
	TreeStats *stats;
};


#endif
