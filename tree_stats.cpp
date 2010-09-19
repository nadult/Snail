#include "rtbase.h"
#include "gfxlib_texture.h"
#include <stdio.h>

/*
	void MemPattern::Init(int size,int r) {
		if(enabled) {
			res=r; mul=double(res)/double(size);
			data.resize(res);
			for(int n=0;n<res;n++) data[n]=0;
		}
	}
	void MemPattern::Draw(gfxlib::Texture &img) const {
		if(enabled) {
			int width=Min(data.size(),img.width);
			for(int n=0;n<width;n++) {
				int color=data[n];
				int r=Clamp(color/255,0,255),g=Clamp(color,0,255),b=0;
				img.Pixel(n,0,r,g,b); img.Pixel(n,1,r,g,b); img.Pixel(n,2,r,g,b);
				img.Pixel(n,img.height-1-0,r,g,b); img.Pixel(n,img.height-1-1,r,g,b); img.Pixel(n,img.height-1-2,r,g,b);
			}
		}
	} */

	const string TreeStats::GenInfo(int resx,int resy, double msRenderTime, double msBuildTime) const {
		double nPixels = double(resx*resy);

		if(!enabled) {
			char buf[2048];
			sprintf(buf, "ms/frame: %6.2f  MPixels/sec: %6.2f\n", msRenderTime,
					(resx * resy * (1000.0 / msRenderTime)) * 0.000001);
			return string(buf);
		}

		char buf[2048];
		char *p = buf + sprintf(buf, "in:%5.2f it:%5.2f  ms:%6.2f  (%d rays)",
				double(data[0]) / nPixels, double(data[1]) / nPixels, msRenderTime, data[2]);

		if(msBuildTime > 0.0)
			p += sprintf(p, " build:%.2f", msBuildTime);
		
		InputAssert(sizeof(timers) / sizeof(int) >= 8);
		sprintf(p, " all:%d tr:%d (in:%d) sh:%d sm:%d  %d %d %d",
				timers[0], timers[1], timers[2], timers[3], timers[4], timers[5], timers[6], timers[7]);

		return string(buf);
	}
	
	void TreeStats::PrintInfo(int resx,int resy,double msRenderTime,double msBuildTime) const {
		printf("%s\n", GenInfo(resx, resy, msRenderTime, msBuildTime).c_str());
	}

