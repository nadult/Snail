#ifndef RTRACER_TREE_STATS_H
#define RTRACER_TREE_STATS_H

#include <memory.h>

class TreeStats
{
public:
	TreeStats() {
		Init();
	}
	INLINE void Update(const TreeStats &local) {
		colTests+=local.colTests;
		iters+=local.iters;
		runs+=local.runs;
		tracedRays+=local.tracedRays;
		coherent+=local.coherent;
		nonCoherent+=local.nonCoherent;
		breaking+=local.breaking;
		notBreaking+=local.notBreaking;
		intersectOk+=local.intersectOk;
		intersectFail+=local.intersectFail;
		skips+=local.skips;
	}
	void Init() {
		memset(this,0,sizeof(TreeStats));
	}
	double CoherentPercentage() const {
		return 100.0*coherent/double(coherent+nonCoherent);
	}
	double BreakingPercentage() const {
		return 100.0*breaking/double(breaking+notBreaking);
	}
	double IntersectFailPercentage() const {
		return 100.0*intersectFail/double(intersectFail+intersectOk);
	}
	void PrintInfo(int resx,int resy,double cycles,double msRenderTime) {
			double raysPerSec=double(tracedRays)*(1000.0/msRenderTime);
			double nPixels=double(resx*resy);

			printf("isct,iter:%.3f %.3f MCycles/frame:%.2f\tMRays/sec:%.2f\t"
					"Coherency:%.2f%% br:%.2f%% fa:%.2f%% %.0f\n",
					double(colTests)/nPixels,double(iters)/nPixels,
				cycles,raysPerSec*0.000001,CoherentPercentage(),
				BreakingPercentage(),IntersectFailPercentage(),double(skips));
	}

	u32 colTests,iters,runs,tracedRays;
	u32 coherent,nonCoherent;

	u32 breaking,notBreaking;
	u32 intersectOk,intersectFail;
	u32 skips;
};

#endif

