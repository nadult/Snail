#include "shading/material.h"
#include "shading/simple_material.h"
#include "shading/tex_material.h"

namespace shading {
	using namespace sampling;

	Material::~Material() { }

	Material *NewMaterial(const string &texName,bool NDotL) {
		PSampler sampler=sampling::NewSampler(texName);

		return sampler?
					NDotL?	(Material*)new TexMaterial<1>(sampler):
							(Material*)new TexMaterial<0>(sampler)
				:	NDotL?	(Material*)new SimpleMaterial<1>(Vec3f(1.0f,1.0f,1.0f)):
							(Material*)new SimpleMaterial<0>(Vec3f(1.0f,1.0f,1.0f)) ;
	}

}
