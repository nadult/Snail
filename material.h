#ifndef RTRACER_MATERIAL_H
#define RTRACER_MATERIAL_H

#include "rtbase.h"
#include "ray_group.h"
#include "light.h"

namespace shading {

	struct Sample {
		enum { maxLightSamples=4 };

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
		};

		BaseMaterial(int f) :flags(f) { }
		virtual ~BaseMaterial();

		enum { primaryFlags=isct::fShOrig|isct::fInvDir|isct::fPrimary };
		typedef PRayGroup<64,primaryFlags> PRays;

		virtual void Shade64(Sample *__restrict__,const PRays&) const=0;
		virtual void Shade64(Sample *__restrict__,const f32x4b*__restrict_,const PRays&) const=0;

		template <int size,int flags>
		void Shade(Sample *__restrict__,const PRayGroup<size,flags>&) const
			{ ThrowException("Shade<",size,",",flags,"> not implemented"); }

		template <int size,int flags>
		void Shade(Sample *__restrict__,const f32x4b*__restrict__,const PRayGroup<size,flags>&) const
			{ ThrowException("Shade<",size,",",flags,"> not implemented"); }

		char flags;	
	};

	BaseMaterial *NewMaterial(const string &texName);

	template <>
	INLINE void BaseMaterial::Shade<64,BaseMaterial::primaryFlags>
		(Sample *__restrict__ samples,const BaseMaterial::PRays &rays) const
		{ Shade64(samples,rays); }

	template <>
	INLINE void BaseMaterial::Shade<64,BaseMaterial::primaryFlags>
		(Sample *__restrict__ samples,const f32x4b*__restrict__ mask,const BaseMaterial::PRays &rays) const
		{ Shade64(samples,mask,rays); }

	template <bool NDotR=true>
	class SimpleMaterial: public BaseMaterial {
	public:
		SimpleMaterial(const Vec3f &col) :BaseMaterial(0),color(col) { }
		SimpleMaterial() { }

		template <int size,int flags>
		void TShade(Sample *__restrict__ samples,const PRayGroup<size,flags> &rays) const {
			for(int q=0;q<blockSize;q++) {
				Sample &s=samples[q];

				Vec3q diffuse(color),specular(0.0f,0.0f,0.0f);
				if(NDotR) diffuse*=rays.Dir(q)|s.normal;

				s.diffuse=diffuse;
				s.specular=diffuse;
			}
		}
		template <int size,int flags>
		void TShade(Sample *__restrict__ samples,const f32x4b*__restrict__ mask,const PRayGroup<size,flags> &rays) const {
			for(int q=0;q<blockSize;q++) {
				Sample &s=samples[q];

				Vec3q diffuse(color),specular(0.0f,0.0f,0.0f);
				if(NDotR) diffuse*=rays.Dir(q)|s.normal;

				s.diffuse=Condition(mask[q],diffuse,s.diffuse);
				s.specular=Condition(mask[q],diffuse,s.specular);
			}
		}

		void Shade64(Sample *__restrict__ samples,const PRays &rays) const
			{ TShade(samples,rays); }
		void Shade64(Sample *__restrict__ samples,const f32x4b *__restrict__ mask,const PRays &rays) const
			{ TShade(samples,mask,rays); }

	private:
		Vec3f color;
	};

	template <class Sampler,bool NDotR=true>
	class TexMaterial: public BaseMaterial {
	public:
		TexMaterial(const Sampler &tSampler) :BaseMaterial(fTexCoords),sampler(tSampler) { }
		TexMaterial() { }

		template <int size,int flags>
		void TShade(Sample *__restrict__ samples,const PRayGroup<size,flags> &rays) const {

			for(int q=0;q<blockSize;q++) {
				Sample &s=samples[q];

				Vec3q diffuse=sampler(s.texCoord,s.texDiff);
				Vec3q specular(0.0f,0.0f,0.0f);
				if(NDotR) diffuse*=rays.Dir(q)|s.normal;

				s.diffuse=diffuse;
				s.specular=specular;
			}
		}
		template <int size,int flags>
		void TShade(Sample *__restrict__ samples,const f32x4b*__restrict__ mask,const PRayGroup<size,flags> &rays) const {

			for(int q=0;q<blockSize;q++) {
				if(!ForAny(mask[q])) continue;
				Sample &s=samples[q];

				Vec3q diffuse=sampler(s.texCoord,s.texDiff);
				Vec3q specular(0.0f,0.0f,0.0f);
				if(NDotR) diffuse*=rays.Dir(q)|s.normal;

				s.diffuse=Condition(mask[q],diffuse,s.diffuse);
				s.specular=Condition(mask[q],specular,s.specular);
			}
		}

		void Shade64(Sample *__restrict__ samples,const PRays &rays) const
			{ TShade(samples,rays); }
		void Shade64(Sample *__restrict__ samples,const f32x4b *__restrict__ mask,const PRays &rays) const
			{ TShade(samples,mask,rays); }

	private:
		Sampler sampler;
	};


}

#endif

