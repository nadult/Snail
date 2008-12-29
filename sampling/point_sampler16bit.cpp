#include "sampling/point_sampler16bit.h"

namespace sampling {

	PointSampler16bit::PointSampler16bit(const gfxlib::Texture &t) :tex(t) {
		if(tex.Width()&(tex.Width()-1)||tex.Height()&(tex.Height()-1))
			ThrowException("Texture width & height must be a power of 2");
		if(tex.GetFormat().GetIdent()!=gfxlib::TI_R5G6B5)
			ThrowException("For now only R5G6B5 textures are supported");
//		if(tex.Mips()>1&&tex.Width()!=tex.Height())
//			ThrowException("Mipmapped textures must have width same as height");

		wMask=tex.Width()-1;
		hMask=tex.Height()-1;
		wMul=tex.Width(); hMul=tex.Height();
		mips=tex.Mips(); w=tex.Width(); h=tex.Height();
		wShift=1; while((1<<wShift)<w) wShift++;
		wShift1=wShift+1;

		if(tex.Pitch()!=(2<<wShift))
			ThrowException("Texture pitch must be equal to width*2");
	}

	Vec3q PointSampler16bit::operator()(const Vec2q &coord) const {
		Vec2q pos=(coord)*Vec2q(wMul,hMul);
		i32x4 x1(pos.x),y1(pos.y);

		x1&=i32x4(wMask);
		y1&=i32x4(hMask);
		y1<<=wShift1;

		const u8 *data=(u8*)tex.DataPointer();

		i32x4 pixels;
		pixels[0]=*(u16*)(data+y1[0]+x1[0]*2);
		pixels[1]=*(u16*)(data+y1[1]+x1[1]*2);
		pixels[2]=*(u16*)(data+y1[2]+x1[2]*2);
		pixels[3]=*(u16*)(data+y1[3]+x1[3]*2);

		floatq b=Shl<3>(pixels&0x001f);
		floatq g=Shr<3>(pixels&0x07e0);
		floatq r=Shr<8>(pixels&0xf800);

		return Vec3q(r,g,b)*f32x4(1.0f/255.0f);
	}

	Vec3q PointSampler16bit::operator()(const Vec2q &coord,const Vec2q &diff) const {
		Vec2q pos=(coord)*Vec2q(wMul,hMul);
		i32x4 x1(pos.x),y1(pos.y);

		uint mip; {	
			floatq min=Min(diff.x*wMul,diff.y*hMul);
			uint pixels=uint(Minimize(min));
			mip=0; while(pixels) { mip++; pixels>>=1; }
			mip=Min(mip,tex.Mips()-1);
		}

		x1>>=mip; y1>>=mip;
		x1&=i32x4(wMask>>mip);
		y1&=i32x4(hMask>>mip);
		y1<<=(wShift1-mip);

		const u8 *data=(u8*)tex.DataPointer(mip);

		i32x4 pixels;
		pixels[0]=*(u16*)(data+y1[0]+x1[0]*2);
		pixels[1]=*(u16*)(data+y1[1]+x1[1]*2);
		pixels[2]=*(u16*)(data+y1[2]+x1[2]*2);
		pixels[3]=*(u16*)(data+y1[3]+x1[3]*2);

		floatq b=Shl<3>(pixels&0x001f);
		floatq g=Shr<3>(pixels&0x07e0);
		floatq r=Shr<8>(pixels&0xf800);

		return Vec3q(r,g,b)*f32x4(1.0f/255.0f);
	}
	
	void PointSampler16bit::Sample(shading::Sample *samples,Cache &c) const {
		for(int k=0;k<shading::blockSize;k++) {
			shading::Sample &s=samples[k];
			s.temp1=operator()(s.texCoord,s.texDiff,c);
		}
	}

}

