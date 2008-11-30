#ifndef RTRACER_SAMPLING_SAT_SAMPLER_H
#define RTRACER_SAMPLING_SAT_SAMPLER_H

#include "rtbase.h"
#include <gfxlib_texture.h>

namespace sampling {

	class SATSampler {
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

		struct Store128bit {
			vector<Sample,AlignedAllocator<Sample> > data;
			Sample *ptr;
			uint w,h,wShift;

			Store128bit() { ptr=0; }
			Store128bit(const vector<Sample,AlignedAllocator<Sample> > &samples,uint w,uint h);
			INLINE const Sample &operator()(uint x,uint y) const { return data[w+x+(y<<wShift)]; }
		};

		Sample ComputeRect(uint ax,uint ay,uint bx,uint by) const;
		void ComputeRect(i32x4 ax,i32x4 ay,i32x4 bx,i32x4 by,Sample out[4]) const;

		Store128bit samples;
		float invW,invH;
		uint w,h,wMask,hMask,wShift;
		Vec3f avg;
	};

}

#endif

