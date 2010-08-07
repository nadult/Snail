#include "sampling.h"
#include "sampling/point_sampler.h"
#include "sampling/point_sampler_dxt.h"
#include "sampling/point_sampler16bit.h"
#include "sampling/sat_sampler.h"
#include <squish.h>
#include <memory.h>
#include <stdio.h>

namespace sampling {

	gfxlib::Texture Compress(const gfxlib::Texture &src) {
		gfxlib::Texture out;

		std::vector<squish::u8> temp(src.Width()*src.Height()*4);
		out.Set(src.Width(),src.Height(),gfxlib::TI_DXT1,5);

		for(int m=0;m<out.Mips();m++) {
			int w=src.Width(m),h=src.Height(m);
			for(int y=0;y<h;y++) {
				squish::u8 *srcPix=((squish::u8*)src.DataPointer(m))+y*w*3;
				for(int x=0;x<w;x++) {
					temp[(x+y*w)*4+0]=srcPix[x*3+2];
					temp[(x+y*w)*4+1]=srcPix[x*3+1];
					temp[(x+y*w)*4+2]=srcPix[x*3+0];
					temp[(x+y*w)*4+3]=255;
				}
			}
			squish::CompressImage(&temp[0],w,h,(squish::u8*)out.DataPointer(m),squish::kDxt1);
			printf("%d ",m); fflush(stdout);
		}

		return out;
	}

	Sampler *NewSampler(PTexture tex) {
		if(!tex) return 0;

		switch(tex->GetFormat().GetIdent()) {
			case gfxlib::TI_A8R8G8B8: {
				PTexture tmp = new gfxlib::Texture(tex->Width(),tex->Height(),gfxlib::TI_R8G8B8,0);
				int w=tex->Width(),h=tex->Height();
				for(int y=0;y<h;y++) {
					u8 *src=(u8*)tex->DataPointer()+y*tex->Pitch();
					u8 *dst=(u8*)tmp->DataPointer()+y*tmp->Pitch();
					for(int x=0;x<w;x++) {
						dst[0]=src[0];
						dst[1]=src[1];
						dst[2]=src[2];
						dst+=3; src+=4;
					}
				}
			 	tmp->GenMips();
				return new PointSampler(tmp);
				return new PointSamplerDXT(Compress(*tmp));
			}
			case gfxlib::TI_R8G8B8:
				if(tex->Mips() == 1) {
					tex->ReallocMips(0);
					tex->GenMips();
				}
				return new PointSampler(tex);
				return new PointSamplerDXT(Compress(*tex));
			case gfxlib::TI_R5G6B5:
				return new PointSampler16bit(*tex);
			case gfxlib::TI_DXT1:
				return new PointSamplerDXT(*tex);
		}

		ThrowException("Cannot create a sampler from texture ", "TODO: name",
						": format is not supported");
	}

}
