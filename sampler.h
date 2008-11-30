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

protected:
	gfxlib::Texture tex;
	uint wMask,hMask;
	float hMul,wMul;
};

class SATSampler: public Sampler {
public:
	SATSampler(const gfxlib::Texture&);
	SATSampler() { }

	Vec3f operator()(const Vec2f &uv,const Vec2f &diff) const;
	Vec3q operator()(const Vec2q &uv,const Vec2q &diff) const;

	// It has to be full, otherwise ray differentials will be wrong
	static Vec2f ComputeDiff(const Vec2q &uv) { return Maximize(uv)-Minimize(uv); }
		
private:
	struct Sample {
		i32x4 sum;

		int R() const { return sum[0]; }
		int G() const { return sum[1]; }
		int B() const { return sum[2]; }

		Sample() { }
		Sample(int rr,int gg,int bb) :sum(rr,gg,bb,0) { }
		Sample(const i32x4 &v) :sum(v) { }

		INLINE void operator-=(const Sample &s) { sum-=s.sum; }
		INLINE void operator+=(const Sample &s) { sum+=s.sum; }
		INLINE Sample operator+(const Sample &s) const { return Sample(sum+s.sum); }
		INLINE Sample operator-(const Sample &s) const { return Sample(sum-s.sum); }
	};

	Sample ComputeRect(uint ax,uint ay,uint bx,uint by) const;

	vector<Sample,AlignedAllocator<Sample> > samples;
	float invW,invH;
	uint w,h,wShift;
	Vec3f avg;
};

#endif

