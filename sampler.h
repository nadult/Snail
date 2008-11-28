#ifndef RTRACER_SAMPLER_H
#define RTRACER_SAMPLER_H

#include "rtbase.h"
#include <gfxlib_texture.h>

class Sampler {
public:
	Sampler(const gfxlib::Texture&);
	Sampler() { }

//	template<class Vec2>
//	Vec3<typename Vec2::TScalar> operator[](const Vec2 &uv) const {
//
//	}

	Vec3f operator()(const Vec2f &uv) const;
	Vec3q operator()(const Vec2q &uv) const;

	gfxlib::Texture tex;
};

#endif

