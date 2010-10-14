#include "shading/uber_material.h"

namespace shading {

	UberMaterial::UberMaterial(const MaterialDesc &desc) :Material(0) {
		ambient = desc.ambient;
		diffuse = desc.diffuse;
		specular = desc.specular;
		refrIndex = desc.refractionIndex;
		Swap(diffuse.x, diffuse.z);
	}
}
