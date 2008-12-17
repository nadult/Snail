#include "material.h"
#include "sampling/point_sampler.h"
#include "sampling/point_sampler_dxt.h"
#include "sampling/point_sampler16bit.h"

namespace shading {
	using namespace sampling;

	BaseMaterial::~BaseMaterial() { }

	BaseMaterial *NewMaterial(const string &texName) {
		if(texName=="") return new SimpleMaterial<1>(Vec3f(1.0f,1.0f,1.0f));
		gfxlib::Texture tex;
		Loader(texName) & tex;


		switch(tex.GetFormat().GetIdent()) {
			case gfxlib::TI_A8R8G8B8: {
				gfxlib::Texture tmp(tex.Width(),tex.Height(),gfxlib::TI_R8G8B8,0);
				int w=tex.Width(),h=tex.Height();
				for(int y=0;y<h;y++) {
					u8 *src=(u8*)tex.DataPointer()+y*tex.Pitch();
					u8 *dst=(u8*)tmp.DataPointer()+y*tmp.Pitch();
					for(int x=0;x<w;x++) {
						dst[0]=src[0];
						dst[1]=src[1];
						dst[2]=src[2];
						dst+=3; src+=4;
					}
				}
			 	tmp.GenMips();
				return new TexMaterial<PointSampler,1>(tmp);
				}
			case gfxlib::TI_R8G8B8:
				if(tex.Mips()==1) {
					tex.ReallocMips(0);
					tex.GenMips();
				}
				return new TexMaterial<PointSampler,1>(tex);
			case gfxlib::TI_R5G6B5: return new TexMaterial<PointSampler16bit,1>(tex);
			case gfxlib::TI_DXT1: return new TexMaterial<PointSamplerDXT,1>(tex);
		}

		ThrowException("Cannot create a material with texture ",texName,": format is not supported");
	}

}
