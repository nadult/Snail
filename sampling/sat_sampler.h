#ifndef RTRACER_SAMPLING_SAT_SAMPLER_H
#define RTRACER_SAMPLING_SAT_SAMPLER_H

#include "rtbase.h"
#include <gfxlib_texture.h>
#include "sampling.h"

namespace sampling {

	class Cache;

	class SATSampler: public Sampler {
	public:
		SATSampler(const gfxlib::Texture&);
		SATSampler() { }

		Vec3f operator()(const Vec2f &uv,const Vec2f &diff) const;
		Vec3q operator()(const Vec2q &uv,const Vec2q &diff) const;
		INLINE Vec3q operator()(const Vec2q &uv,const Vec2q &diff,Cache&) const { return operator()(uv,diff); }

		// It has to be full, otherwise ray differentials will be wrong
		static Vec2f ComputeDiff(const Vec2q &uv) { return Maximize(uv)-Minimize(uv); }
		
		void Sample(shading::Sample *samples,Cache&) const {
			for(int k=0;k<4;k++) {
				shading::Sample &s=samples[k];
				s.temp1=operator()(s.texCoord,s.texDiff);
			}
		}
			
	private:
		struct TSample {
			i32x4 sum;

			int R() const { return sum[0]; }
			int G() const { return sum[1]; }
			int B() const { return sum[2]; }

			TSample() { }
			TSample(int rr,int gg,int bb) :sum(rr,gg,bb,0) { }
			TSample(const i32x4 &v) :sum(v) { }

			INLINE void operator-=(const TSample &s) { sum-=s.sum; }
			INLINE void operator+=(const TSample &s) { sum+=s.sum; }
			INLINE TSample operator+(const TSample &s) const { return TSample(sum+s.sum); }
			INLINE TSample operator-(const TSample &s) const { return TSample(sum-s.sum); }
		};

		INLINE const TSample &Get(uint x,uint y) const { return samples[w+x+(y<<wShift)]; }

		TSample ComputeRect(uint ax,uint ay,uint bx,uint by) const;
		void ComputeRect(i32x4 ax,i32x4 ay,i32x4 bx,i32x4 by,TSample out[4]) const;

		vector<TSample,AlignedAllocator<TSample> > samples;
		uint w,h,wMask,hMask,wShift;
		Vec3f avg;
	};

}

#endif

