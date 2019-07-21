#pragma once

#include "shading/material.h"

namespace shading {

	template <bool NDotR = true>
	class SimpleMaterial: public Material {
	public:
		SimpleMaterial(const Vec3f &col) :Material(0), color(col) { }
		SimpleMaterial() { }

		template <bool sharedOrigin, bool hasMask>
		void Shade_(Sample *__restrict__ samples, const RayGroup<sharedOrigin, hasMask> &rays, SCache &sc) const {
			for(int q = 0; q < blockSize; q++) {
				Sample &s = samples[q];

				Vec3q diffuse(color), specular(0.0f,0.0f,0.0f);
				if(NDotR) diffuse *= Abs(rays.Dir(q) | s.normal);

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

	private:
		Vec3f color;
	};

}
