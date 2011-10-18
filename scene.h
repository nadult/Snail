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
#include <tr1/random>

class Photon;
class PhotonNode;

template <class AccStruct>
class Scene {
public:
	Scene();

	AccStruct geometry;
	Vec3f ambientLight;
	vector<Light> lights;
	vector<shading::PMaterial> materials;
	shading::MatDict matDict;
	shading::TexDict texDict;
	shading::MaterialWrapper<shading::SimpleMaterial<true> > defaultMat;

	vector<Photon> *photons;
	vector<PhotonNode> *photonNodes;
	
	mutable std::tr1::variate_generator<std::tr1::mt19937, std::tr1::uniform_real<> > rand;

	void UpdateMaterials();
	const std::map<string, int> GetMatIdMap() const;

	template <bool sharedOrigin, bool hasMask>
	const TreeStats RayTrace(const RayGroup<sharedOrigin, hasMask>&, Cache&, Vec3q *outColor)
		const __attribute__((noinline));

	enum { numGIRays = 64, giDim = 8 };
	Vec3f giRays[numGIRays];

private:
	const TreeStats TraceLight(RaySelector, const shading::Sample*, Vec3q*__restrict__, Vec3q*__restrict__, int)
							const NOINLINE;

	const TreeStats TraceReflection(RaySelector, const Vec3q*, const shading::Sample*, Cache&, Vec3q *__restrict__)
		const NOINLINE;

	const TreeStats TraceAmbientOcclusion(RaySelector, shading::Sample*, const Vec3q*, Cache&) const;
	
	template <bool sharedOrigin, bool hasMask>
	const TreeStats TraceTransparency(RaySelector, const RayGroup<sharedOrigin, hasMask>&, const floatq*,
			Vec3q *__restrict__, Cache&) const NOINLINE;
};

#endif

