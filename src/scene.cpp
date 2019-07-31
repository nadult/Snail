#include "scene.h"
#include <algorithm>
#include <iostream>

template <class AccStruct>
Scene<AccStruct>::Scene() :defaultMat(Vec3f(1, 1, 1)),
	rand(std::tr1::mt19937(123), std::tr1::uniform_real<>(0.0, 1.0)) {

	ambientLight = Vec3f(0.1f,0.1f,0.1f);
}

template <class AccStruct>
void Scene<AccStruct>::UpdateMaterials() {
	materials.clear();
	for(shading::MatDict::iterator it = matDict.begin(); it != matDict.end(); ++it)
		it->second->id = ~0;
	for(shading::MatDict::iterator it = matDict.begin(); it != matDict.end(); ++it) {
		it->second->id = materials.size();
		materials.push_back(it->second);
	}
}

template <class AccStruct>
const std::map<string, int> Scene<AccStruct>::GetMatIdMap() const {
	std::map<string, int> out;
	for(shading::MatDict::const_iterator it = matDict.begin(); it != matDict.end(); ++it)
		out[it->first] = it->second->id;
	return out;
}

#include "bvh/tree.h"
#include "dbvh/tree.h"

template Scene<BVH>::Scene();
template Scene<DBVH>::Scene();

template void Scene<BVH>::UpdateMaterials();
template void Scene<DBVH>::UpdateMaterials();

template const std::map<string, int> Scene<BVH>::GetMatIdMap() const;
template const std::map<string, int> Scene<DBVH>::GetMatIdMap() const;
