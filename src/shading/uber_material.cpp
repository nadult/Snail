#include "shading/uber_material.h"

namespace shading {

	UberMaterial::UberMaterial(const MaterialDesc &desc) :Material(desc.dissolveFactor > 0?fTransparency : 0) {
		ambient = desc.ambient;
		diffuse = desc.diffuse;
		specular = desc.specular;
		refrIndex = desc.refractionIndex;
		dissFactor = desc.dissolveFactor;
		Swap(diffuse.x, diffuse.z);
	}
}
