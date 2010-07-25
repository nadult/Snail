#ifndef RTRACER_SCENE_H
#define RTRACER_SCENE_H

#include "rtbase.h"
#include "light.h"
#include "shading.h"
#include "formats/loader.h"
#include "sampling.h"
#include "shading/simple_material.h"

typedef TriCache<ShTriangle> ShTriCache;

class Cache {
public:
	Cache() :supersampling(0), reflections(0), transp(0) { }

	int reflections, transp;
	ShTriCache shTriCache;
	sampling::Cache samplingCache;
	bool supersampling;
};

#include "shading/material.h"
#include "shading/simple_material.h"

template <class AccStruct>
class Scene {
public:
	Scene() :defaultMat(Vec3f(1, 1, 1)) {
		ambientLight = Vec3f(0.1f,0.1f,0.1f);
	}

	AccStruct geometry;
	Vec3f ambientLight;
	vector<Light> lights;
	vector<shading::PMaterial> materials;
	shading::MatDict matDict;
	shading::MaterialWrapper< shading::SimpleMaterial<true> > defaultMat;

	void UpdateMaterials() {
		materials.clear();
		for(shading::MatDict::iterator it = matDict.begin(); it != matDict.end(); ++it)
			it->second->id = ~0;
		for(shading::MatDict::iterator it = matDict.begin(); it != matDict.end(); ++it) {
			it->second->id = materials.size();
			materials.push_back(it->second);
		}
	}

	const std::map<string, int> GetMatIdMap() const {
		std::map<string, int> out;
		for(shading::MatDict::const_iterator it = matDict.begin(); it != matDict.end(); ++it)
			out[it->first] = it->second->id;
		return out;
	}

	template <bool sharedOrigin, bool hasMask>
	const TreeStats RayTrace(const RayGroup<sharedOrigin, hasMask>&, Cache&, Vec3q *outColor)
		const __attribute__((noinline));

private:
	const TreeStats TraceLight(RaySelector, const shading::Sample*, Vec3q*__restrict__, Vec3q*__restrict__, int)
							const NOINLINE;

	const TreeStats TraceReflection(RaySelector, const Vec3q*, const shading::Sample*, Cache&, Vec3q *__restrict__)
		const NOINLINE;
	
	template <bool sharedOrigin, bool hasMask>
	const TreeStats TraceTransparency(RaySelector, const RayGroup<sharedOrigin, hasMask>&, const floatq*,
			Vec3q *__restrict__, Cache&) const NOINLINE;
};

#endif

