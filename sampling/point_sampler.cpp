#include "sampling/point_sampler.h"

namespace sampling {

	PointSampler::PointSampler(const gfxlib::Texture &t) :tex(t) {
		if(tex.Width()&(tex.Width()-1)||tex.Height()&(tex.Height()-1))
			ThrowException("Texture width & height must be a power of 2");
		if(tex.GetFormat().GetIdent()!=gfxlib::TI_R8G8B8)
			ThrowException("For now only R8G8B8 textures are supported");
//		if(tex.Mips()>1&&tex.Width()!=tex.Height())
//			ThrowException("Mipmapped textures must have width same as height");

		wMask=tex.Width()-1;
		hMask=tex.Height()-1;
		wMul=tex.Width()-1; hMul=tex.Height()-1;
		mips=tex.Mips(); w=tex.Width(); h=tex.Height();
		wShift=1; while((1<<wShift)<w) wShift++;
	}

	template <class Int,class Vec2>
	INLINE Vec2 ClampTexCoord(const Vec2 &coord) {
		Vec2 uv=coord-Vec2(Int(coord.x),Int(coord.y));
		uv.x=Condition(uv.x<0.0f,uv.x+1.0f,uv.x);
		uv.y=Condition(uv.y<0.0f,uv.y+1.0f,uv.y);
		return uv;
	}

	Vec3f PointSampler::operator()(const Vec2f &coord) const {
		Vec2f pos=coord*Vec2f(wMul,hMul);
		int x1(pos.x),y1(pos.y);

		const u8 *data=(u8*)tex.DataPointer();
		int pitch=tex.Pitch();

		x1&=wMask; y1&=hMask; // in case of NANs etc
		x1*=3;
		y1*=pitch;

		return Vec3f(data[x1+2+y1],data[x1+1+y1],data[x1+0+y1])*(1.0f/255.0f);
	}

	Vec3q PointSampler::operator()(const Vec2q &coord) const {
		Vec2q pos=coord*Vec2q(wMul,hMul);
		i32x4 x1(pos.x),y1(pos.y);
		x1&=i32x4(wMask); y1&=i32x4(hMask);
		x1=x1+x1+x1;

		const u8 *data=(u8*)tex.DataPointer();
		int pitch=tex.Pitch();

		y1[0]*=pitch; y1[1]*=pitch; y1[2]*=pitch; y1[3]*=pitch;

		floatq r=floatq(data[x1[0]+y1[0]+0],data[x1[1]+y1[1]+0],data[x1[2]+y1[2]+0],data[x1[3]+y1[3]+0]);
		floatq g=floatq(data[x1[0]+y1[0]+1],data[x1[1]+y1[1]+1],data[x1[2]+y1[2]+1],data[x1[3]+y1[3]+1]);
		floatq b=floatq(data[x1[0]+y1[0]+2],data[x1[1]+y1[1]+2],data[x1[2]+y1[2]+2],data[x1[3]+y1[3]+2]);

		return Vec3q(b,g,r)*f32x4(1.0f/255.0f);
	}

	Vec3q PointSampler::operator()(const Vec2q &coord,const Vec2q &diff) const {
		Vec2q pos=(coord)*Vec2q(wMul,hMul);
		i32x4 x(pos.x),y(pos.y);

		uint mip; {	
			floatq min=Min(diff.x*wMul,diff.y*hMul);
			uint pixels=uint(Minimize(min));
			mip=0; while(pixels) { mip++; pixels>>=1; }
			mip=Min(mip,tex.Mips()-1);
		}

		const u8 *data=(u8*)tex.DataPointer(mip);
		int pitch=tex.Pitch(mip);
		
		x>>=mip; y>>=mip;
		x&=i32x4(wMask>>mip);
		y&=i32x4(hMask>>mip);

		x=x+x+x;
		y[0]*=pitch; y[1]*=pitch;
		y[2]*=pitch; y[3]*=pitch;

		floatq r=floatq(data[x[0]+y[0]+0],data[x[1]+y[1]+0],data[x[2]+y[2]+0],data[x[3]+y[3]+0]);
		floatq g=floatq(data[x[0]+y[0]+1],data[x[1]+y[1]+1],data[x[2]+y[2]+1],data[x[3]+y[3]+1]);
		floatq b=floatq(data[x[0]+y[0]+2],data[x[1]+y[1]+2],data[x[2]+y[2]+2],data[x[3]+y[3]+2]);

		return Vec3q(b,g,r)*f32x4(1.0f/255.0f);
	}

	// bilinear filtering
	void PointSampler::Sample(shading::Sample *samples,Cache&) const {
		for(int k=0;k<4;k++) {
			shading::Sample &s=samples[k];

			Vec2q pos=ClampTexCoord<i32x4>(s.texCoord)*Vec2q(wMul,hMul);

			uint mip; {	
				floatq min=Min(s.texDiff.x*wMul,s.texDiff.y*hMul);
				uint pixels=uint(Minimize(min)*0.6f);
				mip=0; while(pixels) { mip++; pixels>>=1; }
				mip=Min(mip,tex.Mips()-1);
			}

			const u8 *data=(u8*)tex.DataPointer(mip);
			int pitch=tex.Pitch(mip);

			i32x4 x1(pos.x),y1(pos.y);
			i32x4 x2=x1+i32x4(1),y2=y1+i32x4(1);

			floatq dx=pos.x-floatq(x1),dy=pos.y-floatq(y1);

			x1>>=mip; y1>>=mip;
			x2>>=mip; y2>>=mip;

			x1&=i32x4(wMask>>mip); y1&=i32x4(hMask>>mip);
			x2&=i32x4(wMask>>mip); y2&=i32x4(hMask>>mip);
			x1=x1+x1+x1; x2=x2+x2+x2;

			y1[0]*=pitch; y1[1]*=pitch; y1[2]*=pitch; y1[3]*=pitch;
			y2[0]*=pitch; y2[1]*=pitch; y2[2]*=pitch; y2[3]*=pitch;

			i32x4 o[4]={x1+y1,x2+y1,x1+y2,x2+y2};

#define R(a,b) data[o[a][b]+0]
#define G(a,b) data[o[a][b]+1]
#define B(a,b) data[o[a][b]+2]

			floatq r[4]; {
				r[0]=floatq(R(0,0),R(0,1),R(0,2),R(0,3));
				r[1]=floatq(R(1,0),R(1,1),R(1,2),R(1,3));
				r[2]=floatq(R(2,0),R(2,1),R(2,2),R(2,3));
				r[3]=floatq(R(3,0),R(3,1),R(3,2),R(3,3));
				r[0]=Lerp(Lerp(r[0],r[1],dx),Lerp(r[2],r[3],dx),dy);
			}
			floatq g[4]; {
				g[0]=floatq(G(0,0),G(0,1),G(0,2),G(0,3));
				g[1]=floatq(G(1,0),G(1,1),G(1,2),G(1,3));
				g[2]=floatq(G(2,0),G(2,1),G(2,2),G(2,3));
				g[3]=floatq(G(3,0),G(3,1),G(3,2),G(3,3));
				g[0]=Lerp(Lerp(g[0],g[1],dx),Lerp(g[2],g[3],dx),dy);
			}
			floatq b[4]; {
				b[0]=floatq(B(0,0),B(0,1),B(0,2),B(0,3));
				b[1]=floatq(B(1,0),B(1,1),B(1,2),B(1,3));
				b[2]=floatq(B(2,0),B(2,1),B(2,2),B(2,3));
				b[3]=floatq(B(3,0),B(3,1),B(3,2),B(3,3));
				b[0]=Lerp(Lerp(b[0],b[1],dx),Lerp(b[2],b[3],dx),dy);
			}
#undef R
#undef G
#undef B

			s.temp1=Vec3q(b[0],g[0],r[0])*f32x4(1.0f/255.0f);
		}
	}

	// filtering disabled
/*	void PointSampler::Sample(shading::Sample *samples,Cache&) const {

		uint mip; {	
			floatq min=Min(samples[0].texDiff.x*wMul,samples[0].texDiff.y*hMul);
			uint pixels=uint(Minimize(min));
			mip=0; while(pixels) { mip++; pixels>>=1; }
			mip=Min(mip,tex.Mips()-1);
		}

		const u8 *data=(u8*)tex.DataPointer(mip);
		int pitch=tex.Pitch(mip);
		
		for(int k=0;k<4;k++) {
			shading::Sample &s=samples[k];

			Vec2q pos=(s.texCoord)*Vec2q(wMul,hMul);
			i32x4 x(pos.x),y(pos.y);


			x>>=mip; y>>=mip;
			x&=i32x4(wMask>>mip);
			y&=i32x4(hMask>>mip);

			x=x+x+x;
			y[0]*=pitch; y[1]*=pitch;
			y[2]*=pitch; y[3]*=pitch;

			floatq r=floatq(data[x[0]+y[0]+0],data[x[1]+y[1]+0],data[x[2]+y[2]+0],data[x[3]+y[3]+0]);
			floatq g=floatq(data[x[0]+y[0]+1],data[x[1]+y[1]+1],data[x[2]+y[2]+1],data[x[3]+y[3]+1]);
			floatq b=floatq(data[x[0]+y[0]+2],data[x[1]+y[1]+2],data[x[2]+y[2]+2],data[x[3]+y[3]+2]);

			s.temp1=Vec3q(b,g,r)*f32x4(1.0f/255.0f);
		}
	}*/

}

