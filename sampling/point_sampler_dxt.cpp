#include "sampling/point_sampler_dxt.h"

namespace sampling {

	PointSamplerDXT::PointSamplerDXT(const gfxlib::Texture &t) :tex(t) {
		if(tex.Width()&(tex.Width()-1)||tex.Height()&(tex.Height()-1))
			ThrowException("Texture width & height must be a power of 2");
		if(tex.GetFormat().GetIdent()!=gfxlib::TI_DXT1)
			ThrowException("DXT sampler requires DXT1 texture");
//		if(tex.Mips()>1&&tex.Width()!=tex.Height())
//			ThrowException("Mipmapped textures must have width same as height");

		wMask=tex.Width()-1;
		hMask=tex.Height()-1;
		wMul=tex.Width(); hMul=tex.Height();
		mips=tex.Mips(); w=tex.Width(); h=tex.Height();
		bShift=1; while((1<<bShift)<w) bShift++;
		bShift-=2;
	}

	struct Block { u16 c1,c2; u32 bits; };

	namespace {

		// returns in format R8G8B8 multiplied by 3
		INLINE i32x4 Decompress(const Block *blocks,const i32x4 &bx,const i32x4 &by,
								const i32x4 &xbit,const i32x4 &ybit) {
			i32x4 out;

			for(int k=0;k<4;k++) {
				const Block &block=blocks[bx[k]+by[k]];
				u32 c[4]; //R8G8B8
				c[0]=((block.c1&0xf800)<<8)|((block.c1&0x07e0)<<5)|((block.c1&0x1f)<<3);
				c[1]=((block.c2&0xf800)<<8)|((block.c2&0x07e0)<<5)|((block.c2&0x1f)<<3);

				/*if(block.c1>block.c2)*/ {
					c[2]=c[0]+c[0]+c[1];
					c[3]=c[0]+c[1]+c[1];
				}
			/*	else { // premultiplied alpha
					ThrowException("DXT1 with premultiplied alpha not supported");
				} */
			
				c[0]=c[0]+c[0]+c[0];
				c[1]=c[1]+c[1]+c[1];

				uint shift=(xbit[k]<<1)+(ybit[k]<<3);
				out[k]=c[(block.bits>>shift)&0x3];
			}

			return out;
		}

	}

	Vec3q PointSamplerDXT::operator()(const Vec2q &coord) const {
		Vec2q pos=(coord)*Vec2q(wMul,hMul);
		i32x4 x(pos.x),y(pos.y);

		x&=i32x4(wMask);
		y&=i32x4(hMask);

		const Block *blocks=(Block*)tex.DataPointer();

		i32x4 bx=Shr<2>(x),by=Shr<2>(y)<<bShift;
		i32x4 xbit=x&i32x4(3),ybit=y&i32x4(3);
		i32x4 pixels=Decompress(blocks,bx,by,xbit,ybit);

		floatq r=Shr<16>(pixels);
		floatq g=Shr<8>(pixels&0x3ff00);
		floatq b=pixels&0x3ff;

		return Vec3q(r,g,b)*f32x4(1.0f/(255.0f*3.0f));
	}

	Vec3q PointSamplerDXT::operator()(const Vec2q &coord,const Vec2q &diff) const {
		Vec2q pos=(coord)*Vec2q(wMul,hMul);
		i32x4 x(pos.x),y(pos.y);

		uint mip; {	
			floatq min=Min(diff.x*wMul,diff.y*hMul);
			uint pixels=uint(Minimize(min));
			mip=0; while(pixels) { mip++; pixels>>=1; }
			mip=Min(mip,tex.Mips()-1);
		}

		x>>=mip; y>>=mip;
		x&=i32x4(wMask>>mip);
		y&=i32x4(hMask>>mip);

		const Block *blocks=(Block*)tex.DataPointer(mip);

		i32x4 bx=Shr<2>(x),by=Shr<2>(y)<<(bShift-mip);
		i32x4 xbit=x&i32x4(3),ybit=y&i32x4(3);
		i32x4 pixels=Decompress(blocks,bx,by,xbit,ybit);

		floatq r=Shr<16>(pixels);
		floatq g=Shr<8>(pixels&0x3ff00);
		floatq b=pixels&0x3ff;

		return Vec3q(r,g,b)*f32x4(1.0f/(255.0f*3.0f));
	}

}

