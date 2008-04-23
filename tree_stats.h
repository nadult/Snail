#ifndef RTRACER_TREE_STATS_H
#define RTRACER_TREE_STATS_H

#include <memory.h>

class TreeStats
{
public:
	enum { enabled=1 };

	inline TreeStats() {
		Init();
	}
	inline void Update(const TreeStats &local) {
		if(enabled) {
			intersects+=local.intersects;
			iters+=local.iters;
			tracedRays+=local.tracedRays;
			coherent+=local.coherent;
			nonCoherent+=local.nonCoherent;
			breaking+=local.breaking;
			notBreaking+=local.notBreaking;

			intersectPass+=local.intersectPass;
			intersectFail+=local.intersectFail;

			skips+=local.skips;
		}
	}

	inline void Init() {
		if(enabled) memset(this,0,sizeof(TreeStats));
	}
	double CoherentPercentage() const {
		return enabled?100.0*coherent/double(coherent+nonCoherent):0;
	}
	double BreakingPercentage() const {
		return enabled?100.0*breaking/double(breaking+notBreaking):0;
	}
	double IntersectFailPercentage() const {
		return enabled?100.0*intersectFail/double(intersectFail+intersectPass):0;
	}

	uint TracedRays() const { return enabled?tracedRays:0; }
	uint Intersects() const { return enabled?intersects:0; }
	uint LoopIters() const { return enabled?iters:0; }

	void PrintInfo(int resx,int resy,double msRenderTime) {
			double raysPerSec=double(tracedRays)*(1000.0/msRenderTime);
			double nPixels=double(resx*resy);

			if(!enabled) {
				printf("MSec/frame:%6.2f  MPixels/sec:%6.2f\n",msRenderTime,(resx*resy*(1000.0/msRenderTime))*0.000001);
				return;
			}
			printf("isct,iter:%5.2f %5.2f  MSec/frame:%6.2f  MRays/sec:%5.2f  "
					"Coherency:%.2f%% br:%.2f%% fa:%.2f%% %.0f\n",
					double(intersects)/nPixels,double(iters)/nPixels,
					msRenderTime,raysPerSec*0.000001,CoherentPercentage(),
					BreakingPercentage(),IntersectFailPercentage(),double(tracedRays));
	}

	inline void Breaking(uint val=1) { if(enabled) breaking+=val; }
	inline void NotBreaking(uint val=1) { if(enabled) notBreaking+=val; }

	inline void LoopIteration(uint val=1) { if(enabled) iters+=val; }
	inline void Intersection(uint val=1) { if(enabled) intersects+=val; }
	inline void Skip(uint val=1) { if(enabled) skips++; }

	inline void IntersectFail(uint val=1) { if(enabled) intersectFail+=val; }
	inline void IntersectPass(uint val=1) { if(enabled) intersectPass+=val; }

	inline void TracingRay(uint val=1) { if(enabled) { tracedRays+=val; nonCoherent+=val; } }
	inline void TracingPacket(uint val=1) { if(enabled) { tracedRays+=val; coherent+=val; } }


private:
	u32 intersects,iters,tracedRays;
	u32 coherent,nonCoherent;

	u32 breaking,notBreaking;
	u32 intersectPass,intersectFail;
	u32 skips;
};

#endif

