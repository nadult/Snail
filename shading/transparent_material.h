#ifndef RTRACER_SHADING_TRANSPARENT_MATERIAL_H
#define RTRACER_SHADING_TRANSPARENT_MATERIAL_H

#include "shading/material.h"
#include "sampling.h"

namespace shading {

	template <bool NDotR = true>
	class TransparentMaterial: public Material {
	public:
		TransparentMaterial(sampling::PSampler cSampler, sampling::PSampler tSampler)
			:Material(fTexCoords | fTransparency), cSampler(cSampler), tSampler(tSampler) { }
		TransparentMaterial() { }

		template <bool sharedOrigin, bool hasMask>
		void Shade_(Sample *samples, const RayGroup<sharedOrigin, hasMask> &rays, SCache &sc) const {
			cSampler->Sample(samples, sc, 0);
			Vec3q tcol[blockSize];
			for(int q = 0; q < blockSize; q++)
				tcol[q] = samples[q].temp1;
			tSampler->Sample(samples, sc, 0);

			for(int q = 0; q < blockSize; q++) {
				Sample &s = samples[q];

				Vec3q diffuse = tcol[q];
				Vec3q specular(0.0f, 0.0f, 0.0f);
				if(NDotR) diffuse *= rays.Dir(q) | s.normal;

				if(hasMask) {
					s.diffuse = Condition(rays.SSEMask(q), diffuse, s.diffuse);
					s.specular = Condition(rays.SSEMask(q), diffuse, s.specular);
					s.opacity = Condition(rays.SSEMask(q), s.temp1.x, s.opacity);
				}
				else {
					s.diffuse = diffuse;
					s.specular = diffuse;
					s.opacity = s.temp1.x;
				}
			}
		}

	private:
		sampling::PSampler cSampler, tSampler;
	};


}

#endif

