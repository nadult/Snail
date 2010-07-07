#ifndef RTRACER_SCENE_H
#define RTRACER_SCENE_H

#include "rtbase.h"
#include "light.h"
#include "shading.h"
#include "formats/loader.h"
#include "sampling.h"

class Cache {
public:
	Cache() :supersampling(0), reflections(0) { }

	int reflections;
	ShTriCache shTriCache;
	sampling::Cache samplingCache;
	bool supersampling;
};

#include "shading/material.h"
#include "shading/simple_material.h"

template <class AccStruct>
class Scene {
public:
	typedef shading::PMaterial PMaterial;

	Scene() {
		materials.push_back(new shading::MaterialWrapper<shading::SimpleMaterial<1>>(Vec3f(1.0f,1.0f,1.0f)));
		ambientLight=Vec3f(0.1f,0.1f,0.1f);
	}

	// Call this after changes made to any of the attributes
	void Update() {
		materialFlags.resize(materials.size());
		for(int n = 0; n < materials.size(); n++)
			materialFlags[n] = materials[n]->flags;
	}

	AccStruct geometry;
	Vec3f ambientLight;
	vector<Light> lights;
	vector<PMaterial> materials;

	template <bool sharedOrigin, bool hasMask>
	TreeStats<1> RayTrace(const RayGroup<sharedOrigin, hasMask>&, Cache&, Vec3q *outColor)
		const __attribute__((noinline));

private:
	TreeStats<1> TraceLight(RaySelector, const shading::Sample*, Vec3q*__restrict__, Vec3q*__restrict__, int)
							const __attribute__((noinline));

	TreeStats<1> TraceReflection(RaySelector, const Vec3q*, const shading::Sample*, Cache&, Vec3q *__restrict__)
		const __attribute__((noinline));

	vector<char> materialFlags;
};

#endif

