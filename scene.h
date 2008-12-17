#ifndef RTRACER_SCENE_H
#define RTRACER_SCENE_H

#include "rtbase.h"
#include "light.h"
#include "shading.h"
#include "context.h"
#include "formats/loader.h"

template <int size_>
class Result {
public:
	enum { size=size_ };
	Vec3q color[size];
	TreeStats<1> stats;
};

#include "sampling.h"

class Cache {
public:
	ShTriCache shTriCache;
	sampling::Cache samplingCache;
};

#include "material.h"

template <class AccStruct>
class Scene {
public:
	typedef Ptr<shading::BaseMaterial> PMaterial;
		
	Scene() {
		materials.push_back(new shading::SimpleMaterial<1>(Vec3f(1.0f,1.0f,1.0f)));
		ambientLight=Vec3f(0.1f,0.1f,0.1f);
	}

	// Call this after changes made to any of the attributes
	void Update() {
		materialFlags.resize(materials.size());
		for(int n=0;n<materials.size();n++)
			materialFlags[n]=materials[n]->flags;
	}

	AccStruct geometry;
	Vec3f ambientLight;
	vector<Light> lights;
	vector<PMaterial> materials;

	template <int flags,int size,class Selector>
	Result<size> RayTrace(const RayGroup<size,flags>&,const Selector&,Cache&) const NOINLINE;

private:
	template <int size,class Selector>
	TreeStats<1> TraceLight(const Selector&,const shading::Sample*,Vec3q*__restrict__,Vec3q*__restrict__,int) const NOINLINE;

	template <int flags,int size,class Selector>
	Result<size> TraceReflection(const RayGroup<size,flags>&,const Selector&,const Vec3q*,const Vec3q*) const NOINLINE;

	vector<char> materialFlags;
};

#endif

