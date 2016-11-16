#include "sampling/point_sampler_dxt.h"

namespace sampling {

	PointSamplerDXT::PointSamplerDXT(const Texture &t) :tex(t) {
		if(tex.Width()&(tex.Width()-1)||tex.Height()&(tex.Height()-1))
			THROW("Texture width & height must be a power of 2");
		if(tex.GetFormat().GetIdent()!=TextureFormatId::dxt1)
			THROW("DXT sampler requires DXT1 texture");
//		if(tex.Mips()>1&&tex.Width()!=tex.Height())
//			THROW("Mipmapped textures must have width same as height");

		wMask=tex.Width()-1;
		hMask=tex.Height()-1;
		wMul=tex.Width(); hMul=tex.Height();
		mips=tex.Mips(); w=tex.Width(); h=tex.Height();
		bShift=1; while((1<<bShift)<w) bShift++;
		bShift-=2;
	}

	typedef DXTBlock Block;

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
					THROW("DXT1 with premultiplied alpha not supported");
				} */
			
				c[0]=c[0]+c[0]+c[0];
				c[1]=c[1]+c[1]+c[1];

				uint shift=(xbit[k]<<1)+(ybit[k]<<3);
				out[k]=c[(block.bits32>>shift)&0x3];
			}


			return out;
		}

	}

	DXTCache::DXTCache() {
		for(int n=0;n<size;n++) idents[n]=0;
	}

	void DXTCache::Insert(uint hash,Ident ident,int bShift) {
		const Block *b[4];
		b[0]=(const Block*)ident;
		b[1]=b[0]+1;
		b[2]=b[0]+(1<<bShift);
		b[3]=b[1]+(1<<bShift);

		idents[hash]=b[0];
		i32x4 temp[16];

		for(int k=0;k<4;k++) {
			const Block &block=*b[k];
			i32x4 *dst=temp+k*4;

			u32 c[4]; //R8G8B8
			c[0]=((block.c1&0xf800)<<8)|((block.c1&0x07e0)<<5)|((block.c1&0x1f)<<3);
			c[1]=((block.c2&0xf800)<<8)|((block.c2&0x07e0)<<5)|((block.c2&0x1f)<<3);

			c[2]=c[0]+c[0]+c[1];
			c[3]=c[0]+c[1]+c[1];

			c[0]=c[0]+c[0]+c[0];
			c[1]=c[1]+c[1]+c[1];

			for(int t=0;t<4;t++) {
				dst[t][0]=c[block.bits8[t]&0x3];
				dst[t][1]=c[(block.bits8[t]>>2)&0x3];
				dst[t][2]=c[(block.bits8[t]>>4)&0x3];
				dst[t][3]=c[block.bits8[t]>>6];
			}
		}
		
		i32x4 *out=colors[hash];

		out[ 0]=temp[ 0]; out[ 1]=temp[ 4]; out[ 2]=temp[ 1]; out[ 3]=temp[ 5];
		out[ 4]=temp[ 2]; out[ 5]=temp[ 6]; out[ 6]=temp[ 3]; out[ 7]=temp[ 7];
		out[ 8]=temp[ 8]; out[ 9]=temp[12]; out[10]=temp[ 9]; out[11]=temp[13];
		out[12]=temp[10]; out[13]=temp[14]; out[14]=temp[11]; out[15]=temp[15];
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

	Vec3q PointSamplerDXT::operator()(const Vec2q &coord,const Vec2q &diff,Cache &sCache) const {
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

		i32x4 tbx=Shr<3>(x),tby=Shr<3>(y);
		i32x4 pixels;

		const Block *blocks=(Block*)tex.DataPointer(mip);

		if(ForAll(tbx==tbx[0]&&tby==tby[0])) {
			DXTCache &cache=sCache.dxtCache;

			const DXTCache::Ident ident=cache.GetIdent(tbx[0],tby[0],bShift-mip,blocks);
			uint hash=cache.Hash(tbx[0],tby[0],mip);

			if(cache.idents[hash]!=ident)
				cache.Insert(hash,ident,bShift-mip);
				
			const int *block=(int*)cache.colors[hash];
			i32x4 offset=(x&i32x4(7))+Shl<3>(y&i32x4(7));

			pixels[0]=block[offset[0]];
			pixels[1]=block[offset[1]];
			pixels[2]=block[offset[2]];
			pixels[3]=block[offset[3]];
		}
		else {
			i32x4 bx=Shr<2>(x),by=Shr<2>(y)<<(bShift-mip);
			i32x4 xbit=x&i32x4(3),ybit=y&i32x4(3);
			pixels=Decompress(blocks,bx,by,xbit,ybit);
		}
			
		floatq r=Shr<16>(pixels);
		floatq g=Shr<8>(pixels&0x3ff00);
		floatq b=pixels&0x3ff;

		return Vec3q(r,g,b)*f32x4(1.0f/(255.0f*3.0f));
	}
	
	void PointSamplerDXT::Sample(shading::Sample *samples,Cache &c, bool mipmapping) const {
		for(int k=0;k<shading::blockSize;k++) {
			shading::Sample &s=samples[k];
			s.temp1=operator()(s.texCoord,s.texDiff,c);
		}
	}

}

