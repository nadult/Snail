#ifndef RTRACER_SHADING_UBER_MATERIAL_H
#define RTRACER_SHADING_UBER_MATERIAL_H

#include "shading/material.h"
#include "sampling.h"

namespace shading {

	class UberMaterial: public Material {
	public:
		UberMaterial(const MaterialDesc&);

		template <bool sharedOrigin, bool hasMask>
		void Shade_(Sample *samples, const RayGroup<sharedOrigin, hasMask> &rays, SCache &sc) const {
			Vec3q diff(diffuse), spec(specular);

			for(int q = 0; q < blockSize; q++) {
				Sample &s = samples[q];
				Vec3q diffuse = diff * Abs(rays.Dir(q) | s.normal);

				if(hasMask) {
					s.diffuse = Condition(rays.SSEMask(q), diffuse, s.diffuse);
					s.specular = Condition(rays.SSEMask(q), spec, s.specular);
				}
				else {
					s.diffuse = diffuse;
					s.specular = diffuse;
				}
			}
		}

	private:
		Vec3f ambient, diffuse, specular;
		float refrIndex;
	};


}

#endif

