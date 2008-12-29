#ifndef RTRACER_SHADING_MATERIAL_H
#define RTRACER_SHADING_MATERIAL_H

#include "rtbase.h"
#include "ray_group.h"
#include "shading.h"
#include "sampling.h"

namespace shading {

	class Material: public RefCounter {
	public:
		enum {
			fTexCoords = 1,
			fReflection = 2,
			fRefraction = 4,
		};

		typedef RayGroup<4,1> RaysS;
		typedef RayGroup<4,0> Rays;
		typedef sampling::Cache SCache;

		Material(int f) :flags(f) { }
		virtual ~Material();

		virtual void Shade(Sample *__restrict__,const RaysS&,SCache&) const=0;
		virtual void Shade(Sample *__restrict__,const f32x4b*__restrict_,const RaysS&,SCache&) const=0;
		virtual void Shade(Sample *__restrict__,const Rays&,SCache&) const=0;
		virtual void Shade(Sample *__restrict__,const f32x4b*__restrict_,const Rays&,SCache&) const=0;

		char flags;	
	};

	typedef Ptr<Material> PMaterial;

	Material *NewMaterial(const string &texName,bool nDotL=1);

}

#endif

