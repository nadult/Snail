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

		typedef sampling::Cache SCache;

		Material(int f) :flags(f) { }
		virtual ~Material();

		virtual void Shade(Sample*, const RayGroup<0, 0>&, SCache&) const = 0;
		virtual void Shade(Sample*, const RayGroup<0, 1>&, SCache&) const = 0;
		virtual void Shade(Sample*, const RayGroup<1, 0>&, SCache&) const = 0;
		virtual void Shade(Sample*, const RayGroup<1, 1>&, SCache&) const = 0;

		char flags;	
	};

	template <class MatClass>
	class MaterialWrapper: public MatClass {
	public:
		typedef sampling::Cache SCache;

		template<class ...Args>
		MaterialWrapper(const Args ... args) :MatClass(args...) { }

		void Shade(Sample *samples, const RayGroup<0, 0> &rays, SCache &sc) const
			{ MatClass::Shade_(samples, rays, sc); }
		void Shade(Sample *samples, const RayGroup<0, 1> &rays, SCache &sc) const
			{ MatClass::Shade_(samples, rays, sc); }
		void Shade(Sample *samples, const RayGroup<1, 0> &rays, SCache &sc) const
			{ MatClass::Shade_(samples, rays, sc); }
		void Shade(Sample *samples, const RayGroup<1, 1> &rays, SCache &sc) const
			{ MatClass::Shade_(samples, rays, sc); }
	};

	typedef Ptr<Material> PMaterial;

	Material *NewMaterial(const string &texName,bool nDotL = 1);

}

#endif

