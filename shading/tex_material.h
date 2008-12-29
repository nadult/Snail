#ifndef RTRACER_SHADING_TEX_MATERIAL_H
#define RTRACER_SHADING_TEX_MATERIAL_H

#include "shading/material.h"
#include "sampling.h"

namespace shading {

	template <bool NDotR=true>
	class TexMaterial: public Material {
	public:
		TexMaterial(sampling::PSampler tSampler) :Material(fTexCoords),sampler(tSampler) { }
		TexMaterial() { }

		template <class Rays>
		void Shade_(Sample *__restrict__ samples,const Rays &rays,SCache &sc) const {
			sampler->Sample(samples,sc);

			for(int q=0;q<blockSize;q++) {
				Sample &s=samples[q];

				Vec3q diffuse=s.temp1;
				Vec3q specular(0.0f,0.0f,0.0f);
				if(NDotR) diffuse*=rays.Dir(q)|s.normal;

				s.diffuse=diffuse;
				s.specular=specular;
			}
		}
		template <class Rays>
		void Shade_(Sample *__restrict__ samples,const f32x4b*__restrict__ mask,const Rays &rays,SCache &sc) const {
			sampler->Sample(samples,sc);

			for(int q=0;q<blockSize;q++) {
				if(!ForAny(mask[q])) continue;
				Sample &s=samples[q];

				Vec3q diffuse=s.temp1;
				Vec3q specular(0.0f,0.0f,0.0f);
				if(NDotR) diffuse*=rays.Dir(q)|s.normal;

				s.diffuse=Condition(mask[q],diffuse,s.diffuse);
				s.specular=Condition(mask[q],specular,s.specular);
			}
		}

		void Shade(Sample *__restrict__ samples,const Rays &rays,SCache &sc) const
			{ Shade_(samples,rays,sc); }
		void Shade(Sample *__restrict__ samples,const f32x4b*__restrict__ mask,const Rays &rays,SCache &sc) const
			{ Shade_(samples,mask,rays,sc); }
		void Shade(Sample *__restrict__ samples,const RaysS &rays,SCache &sc) const
			{ Shade_(samples,rays,sc); }
		void Shade(Sample *__restrict__ samples,const f32x4b*__restrict__ mask,const RaysS &rays,SCache &sc) const
			{ Shade_(samples,mask,rays,sc); }
	private:
		sampling::PSampler sampler;
	};


}

#endif

