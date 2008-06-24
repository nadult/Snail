#include "tree_stats.h"
#include "image.h"

	void MemPattern::Init(int size,int r) {
		if(enabled) {
			res=r; mul=double(res)/double(size);
			data.resize(res);
			for(int n=0;n<res;n++) data[n]=0;
		}
	}
	void MemPattern::Draw(Image &img) const {
		if(enabled) {
			int width=Min(data.size(),img.width);
			for(int n=0;n<width;n++) {
				int color=data[n];
				int r=Clamp(color/255,0,255),g=Clamp(color,0,255),b=0;
				img.Pixel(n,0,r,g,b);
				img.Pixel(n,1,r,g,b);
				img.Pixel(n,2,r,g,b);
			}
		}
	}
	void TreeStats::PrintInfo(int resx,int resy,double msRenderTime) {
		double raysPerSec=double(tracedRays)*(1000.0/msRenderTime);
		double nPixels=double(resx*resy);

		if(!enabled) {
			printf("MSec/frame:%6.2f  MPixels/sec:%6.2f\n",msRenderTime,(resx*resy*(1000.0/msRenderTime))*0.000001);
			return;
		}
		printf("isct,iter:%5.2f %5.2f  MSec/frame:%6.2f  MRays/sec:%5.2f  "
				/*"Coh:%.2f%% "*/"br:%.2f%% fa:%.2f%% %.0f R:%d\n",
				double(intersects)/nPixels,double(iters)/nPixels,
				msRenderTime,raysPerSec*0.000001/*,Coherent()*100.0f*/,
				TBreaking()*100.0f,TIntersectFail()*100.0f,double(skips),tracedRays);
	}

