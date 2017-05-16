#ifndef RTRACER_SHADING_TEX_MATERIAL_H
#define RTRACER_SHADING_TEX_MATERIAL_H

#include "shading/material.h"
#include "sampling.h"

namespace shading {

	template <bool NDotR = true>
	class TexMaterial: public Material {
	public:
		TexMaterial(sampling::PSampler tSampler) :Material(fTexCoords), sampler(tSampler) { }
		TexMaterial() { }

		template <bool sharedOrigin, bool hasMask>
		void Shade_(Sample *samples, const RayGroup<sharedOrigin, hasMask> &rays, SCache &sc) const {
			sampler->Sample(samples, sc);

			for(int q = 0; q < blockSize; q++) {
				Sample &s = samples[q];

				Vec3q diffuse = s.temp1;
				Vec3q specular(0.0f, 0.0f, 0.0f);
				if(NDotR) diffuse *= rays.Dir(q) | s.normal;

				if(hasMask) {
					s.diffuse = Condition(rays.SSEMask(q), diffuse, s.diffuse);
					s.specular = Condition(rays.SSEMask(q), diffuse, s.specular);
				}
				else {
					s.diffuse = diffuse;
					s.specular = diffuse;
				}
			}
		}
		
		const MipmapTexture *GetTexture() const { return sampler->GetTexture(); }

	private:
		sampling::PSampler sampler;
	};


}

#endif

