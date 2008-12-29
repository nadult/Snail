#ifndef RTRACER_SAMPLING_POINT_SAMPLER_H
#define RTRACER_SAMPLING_POINT_SAMPLER_H

#include "rtbase.h"
#include <gfxlib_texture.h>
#include "sampling.h"

namespace sampling {

	class PointSampler: public Sampler {
	public:
		PointSampler(const gfxlib::Texture&);
		PointSampler() { }

		Vec3f operator()(const Vec2f &uv) const;
		Vec3q operator()(const Vec2q &uv) const;
		Vec3q operator()(const Vec2q &uv,const Vec2q &diff) const;
		INLINE Vec3q operator()(const Vec2q &uv,const Vec2q &diff,Cache&) const { return operator()(uv,diff); }

		void Sample(shading::Sample*,Cache&) const;

	protected:
		gfxlib::Texture tex;
		uint wMask,hMask,mips,w,h,wShift;
		float hMul,wMul;
	};

}

#endif

