#pragma once

#include "rtbase.h"
#include "ray_group.h"
#include "shading.h"
#include "sampling.h"

namespace shading {

	class Material {
	public:
		enum {
			fTexCoords = 1,
			fReflection = 2,
			fRefraction = 4,
			fTransparency = 8,
		};

		typedef sampling::Cache SCache;

		Material(int flags) :flags(flags) { }
		virtual ~Material();

		virtual void Shade(Sample*, const RayGroup<0, 0>&, SCache&) const = 0;
		virtual void Shade(Sample*, const RayGroup<0, 1>&, SCache&) const = 0;
		virtual void Shade(Sample*, const RayGroup<1, 0>&, SCache&) const = 0;
		virtual void Shade(Sample*, const RayGroup<1, 1>&, SCache&) const = 0;

		virtual const MipmapTexture *GetTexture() const { return 0; }

		int id;
		int flags;
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

	struct MaterialDesc {
		MaterialDesc();

		Vec3f ambient, diffuse, specular, emissive;
		Vec3f transmission;
		int illuminationModel;
		float dissolveFactor;
		float specularExponent;
		int refractionIndex;

		string ambientMap, diffuseMap;
		string specularMap, emissiveMap;
		string exponentMap, dissolveMap;

		string name;
	};

	const vector<MaterialDesc> LoadMaterialDescs(const string &fileName);

	using sampling::TexDict;

	using PMaterial = shared_ptr<Material>;
	typedef std::map<string, PMaterial> MatDict;

	PMaterial NewMaterial(PMipmapTexture tex, bool nDotL = 1);

	TexDict LoadTextures(const vector<MaterialDesc> &matDescs, const string &texPath);
	MatDict MakeMaterials(const vector<MaterialDesc> &matDescs, const TexDict &texDict);

}
