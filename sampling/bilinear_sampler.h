#ifndef RTRACER_SAMPLING_BILINEAR_SAMPLER_H
#define RTRACER_SAMPLING_BILINEAR_SAMPLER_H

#include "rtbase.h"
#include "sampling.h"
#include "mipmap_texture.h"

namespace sampling {

	class BilinearSampler {
	public:
		BilinearSampler(const MipmapTexture&);
		BilinearSampler() { }

	//	template<class Vec2>
	//	Vec3<typename Vec2::TScalar> operator[](const Vec2 &uv) const {
	//
	//	}

		Vec3f operator()(const Vec2f &uv) const;
		Vec3q operator()(const Vec2q &uv) const;

	protected:
		MipmapTexture tex;
		uint wMask,hMask;
		float hMul,wMul;
	};

}

#endif

