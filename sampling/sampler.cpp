#include "sampling.h"
#include "sampling/point_sampler.h"
//#include "sampling/point_sampler_dxt.h"
#include "sampling/sat_sampler.h"
#include "mipmap_texture.h"

namespace sampling {

	PSampler NewSampler(PMipmapTexture tex) {
		if(!tex)
			return {};

		switch(tex->GetFormat().id()) {
			case fwk::TextureFormatId::rgba: {
				auto tmp = make_shared<MipmapTexture>(tex->Width(),tex->Height(), fwk::TextureFormatId::rgb, 0);
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
				return make_shared<PointSampler>(PointSampler(tmp));
			}
			case fwk::TextureFormatId::rgb:
				if(tex->Mips() == 1) {
					tex->ReallocMips(0);
					tex->GenMips();
				}
				return make_shared<PointSampler>(tex);
		//	case gfxlib::TI_DXT1:
		//		return make_shared<PointSamplerDXT>(*tex);
		}

		THROW("Cannot create a sampler from texture ", "TODO: name",
						": format is not supported");
		return {};
	}

}
