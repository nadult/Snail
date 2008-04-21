#ifndef RTRACER_TREE_STATS_H
#define RTRACER_TREE_STATS_H

#include <memory.h>

class TreeStats
{
public:
	enum { enabled=1 };

	TreeStats() {
		Init();
	}
	INLINE void Update(const TreeStats &local) {
		if(!enabled) return;

		intersects+=local.intersects;
		iters+=local.iters;
		runs+=local.runs;
		tracedRays+=local.tracedRays;
		coherent+=local.coherent;
		nonCoherent+=local.nonCoherent;
		breaking+=local.breaking;
		notBreaking+=local.notBreaking;

		intersectPass+=local.intersectPass;
		intersectFail+=local.intersectFail;

		skips+=local.skips;
	}

	void Init() {
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

	uint TracedTays() const { return enabled?tracedRays:0; }
	uint Intersects() const { return enabled?intersects:0; }
	uint LoopIters() const { return enabled?iters:0; }

	void PrintInfo(int resx,int resy,double cycles,double msRenderTime) {
			double raysPerSec=double(tracedRays)*(1000.0/msRenderTime);
			double nPixels=double(resx*resy);

			if(!enabled) {
				printf("MCycles/frame:%.2f\tMRays/sec:%.2f\n",cycles,raysPerSec*0.000001);
				return;
			}
			printf("isct,iter:%.3f %.3f MCycles/frame:%.2f\tMRays/sec:%.2f\t"
					"Coherency:%.2f%% br:%.2f%% fa:%.2f%% %.0f\n",
					double(intersects)/nPixels,double(iters)/nPixels,
					cycles,raysPerSec*0.000001,CoherentPercentage(),
					BreakingPercentage(),IntersectFailPercentage(),double(skips));
	}

	inline void LoopIteration(uint val=1) { if(enabled) iters+=val; }
	inline void Intersection(uint val=1) { if(enabled) intersects+=val; }
	inline void Run(uint val=1) { if(enabled) runs+=val; }
	inline void Skip(uint val=1) { if(enabled) skips++; }

	inline void IntersectFail(uint val=1) { if(enabled) intersectFail+=val; }
	inline void IntersectPass(uint val=1) { if(enabled) intersectPass+=val; }

	inline void TracingRay(uint val=1) { if(enabled) tracedRays+=val; }

private:
	u32 intersects,iters,runs,tracedRays;
	u32 coherent,nonCoherent;

	u32 breaking,notBreaking;
	u32 intersectPass,intersectFail;
	u32 skips;
};

#endif

