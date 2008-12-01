#ifndef RTRACER_SAMPLING_POINT_SAMPLER_H
#define RTRACER_SAMPLING_POINT_SAMPLER_H

#include "rtbase.h"
#include <gfxlib_texture.h>

namespace sampling {

	class PointSampler {
	public:
		PointSampler(const gfxlib::Texture&);
		PointSampler() { }

	//	template<class Vec2>
	//	Vec3<typename Vec2::TScalar> operator[](const Vec2 &uv) const {
	//
	//	}

		Vec3f operator()(const Vec2f &uv) const; // biggest mipmap will be used
		Vec3q operator()(const Vec2q &uv) const; // biggest mipmap will be used
		Vec3q operator()(const Vec2q &uv,const Vec2q &diff) const;

	protected:
		gfxlib::Texture tex;
		uint wMask,hMask,mips,w,h,wShift;
		float hMul,wMul;
	};

}

#endif

