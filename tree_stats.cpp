#include "tree_stats.h"


	void TreeStats::PrintInfo(int resx,int resy,double msRenderTime) {
		double raysPerSec=double(tracedRays)*(1000.0/msRenderTime);
		double nPixels=double(resx*resy);

		if(!enabled) {
			printf("MSec/frame:%6.2f  MPixels/sec:%6.2f\n",msRenderTime,(resx*resy*(1000.0/msRenderTime))*0.000001);
			return;
		}
		printf("isct,iter:%5.2f %5.2f  MSec/frame:%6.2f  MRays/sec:%5.2f  "
				"Coherency:%.2f%% br:%.2f%% fa:%.2f%% %.0f R:%d\n",
				double(intersects)/nPixels,double(iters)/nPixels,
				msRenderTime,raysPerSec*0.000001,Coherent()*100.0f,
				TBreaking()*100.0f,TIntersectFail()*100.0f,double(skips),tracedRays);
	}

