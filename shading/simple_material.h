#ifndef RTRACER_SHADING_SIMPLE_MATERIAL_H
#define RTRACER_SHADING_SIMPLE_MATERIAL_H

#include "shading/material.h"

namespace shading {

	template <bool NDotR=true>
	class SimpleMaterial: public Material {
	public:
		SimpleMaterial(const Vec3f &col) :Material(0),color(col) { }
		SimpleMaterial() { }

		template <class Rays>
		void Shade_(Sample *__restrict__ samples,const Rays &rays,SCache &sc) const {
			for(int q=0;q<blockSize;q++) {
				Sample &s=samples[q];

				Vec3q diffuse(color),specular(0.0f,0.0f,0.0f);
				if(NDotR) diffuse*=rays.Dir(q)|s.normal;

				s.diffuse=diffuse;
				s.specular=diffuse;
			}
		}
		template <class Rays>
		void Shade_(Sample *__restrict__ samples,const f32x4b*__restrict__ mask,const Rays &rays,SCache &sc) const {
			for(int q=0;q<blockSize;q++) {
				Sample &s=samples[q];

				Vec3q diffuse(color),specular(0.0f,0.0f,0.0f);
				if(NDotR) diffuse*=rays.Dir(q)|s.normal;

				s.diffuse=Condition(mask[q],diffuse,s.diffuse);
				s.specular=Condition(mask[q],diffuse,s.specular);
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
		Vec3f color;
	};

}

#endif

