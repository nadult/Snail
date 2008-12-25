#ifndef RTRACER_MATERIAL_H
#define RTRACER_MATERIAL_H

#include "rtbase.h"
#include "ray_group.h"
#include "light.h"

namespace sampling { struct Cache; }

namespace shading {

	struct Sample {
		floatq distance;

		Vec3q position;
		Vec3q normal;
		Vec3q reflection;

		Vec2q texCoord;
		Vec2q texDiff;

		Vec3q diffuse;
		Vec3q specular;
	};

	class BaseMaterial: public RefCounter {
	public:
		enum { blockSize=4 };

		enum {
			fTexCoords = 1,
			fReflection = 2,
			fRefraction = 4,
		};

		typedef RayGroup<4,1> RaysS;
		typedef RayGroup<4,0> Rays;
		typedef sampling::Cache SCache;

		BaseMaterial(int f) :flags(f) { }
		virtual ~BaseMaterial();

		virtual void Shade(Sample *__restrict__,const RaysS&,SCache&) const=0;
		virtual void Shade(Sample *__restrict__,const f32x4b*__restrict_,const RaysS&,SCache&) const=0;
		virtual void Shade(Sample *__restrict__,const Rays&,SCache&) const=0;
		virtual void Shade(Sample *__restrict__,const f32x4b*__restrict_,const Rays&,SCache&) const=0;

		char flags;	
	};

	BaseMaterial *NewMaterial(const string &texName,bool nDotL=1);

	template <bool NDotR=true>
	class SimpleMaterial: public BaseMaterial {
	public:
		SimpleMaterial(const Vec3f &col) :BaseMaterial(0),color(col) { }
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

	template <class Sampler,bool NDotR=true>
	class TexMaterial: public BaseMaterial {
	public:
		TexMaterial(const Sampler &tSampler) :BaseMaterial(fTexCoords),sampler(tSampler) { }
		TexMaterial() { }

		template <class Rays>
		void Shade_(Sample *__restrict__ samples,const Rays &rays,SCache &sc) const {
			for(int q=0;q<blockSize;q++) {
				Sample &s=samples[q];

				Vec3q diffuse=sampler(s.texCoord,s.texDiff,sc);
				Vec3q specular(0.0f,0.0f,0.0f);
				if(NDotR) diffuse*=rays.Dir(q)|s.normal;

				s.diffuse=diffuse;
				s.specular=specular;
			}
		}
		template <class Rays>
		void Shade_(Sample *__restrict__ samples,const f32x4b*__restrict__ mask,const Rays &rays,SCache &sc) const {
			for(int q=0;q<blockSize;q++) {
				if(!ForAny(mask[q])) continue;
				Sample &s=samples[q];

				Vec3q diffuse=sampler(s.texCoord,s.texDiff,sc);
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
		Sampler sampler;
	};


}

#endif

