#include "sampling/point_sampler.h"

namespace sampling {

	PointSampler::PointSampler(const gfxlib::Texture &t) :tex(t) {
		if(tex.Width()&(tex.Width()-1)||tex.Height()&(tex.Height()-1))
			ThrowException("Texture width & height must be a power of 2");
		if(tex.GetFormat().GetIdent()!=gfxlib::TI_R8G8B8)
			ThrowException("For now only R8G8B8 textures are supported");

		wMask=tex.Width()-1;
		hMask=tex.Height()-1;
		wMul=tex.Width(); hMul=tex.Height();
		mips=tex.Mips(); w=tex.Width(); h=tex.Height();
		wShift=1; while((1<<wShift)<w) wShift++;
	}

	/*
	template <class Int,class Vec2>
	INLINE Vec2 ClampTexCoord(const Vec2 &coord) {
		Vec2 uv=coord-Vec2(Int(coord.x),Int(coord.y));
		uv.x=Condition(uv.x<0.0f,uv.x+1.0f,uv.x);
		uv.y=Condition(uv.y<0.0f,uv.y+1.0f,uv.y);
		return uv;
	} */

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
		i32x4 x1(pos.x),y1(pos.y);

		uint mip; {	
			floatq min=Min(diff.x*float(tex.Width()),diff.y*float(tex.Height()));
			int pixels=int(Minimize(min));
			mip=0; while(pixels) { mip++; pixels>>=1; }
			mip=Min(mip,tex.Mips()-1);
		}

		const u8 *data=(u8*)tex.DataPointer(mip);
		int pitch=tex.Pitch(mip);
		
		x1>>=mip; y1>>=mip;
		x1&=i32x4(wMask>>mip);
		y1&=i32x4(hMask>>mip);

		x1=x1+x1+x1;
		y1[0]*=pitch; y1[1]*=pitch;
		y1[2]*=pitch; y1[3]*=pitch;

		floatq r=floatq(data[x1[0]+y1[0]+0],data[x1[1]+y1[1]+0],data[x1[2]+y1[2]+0],data[x1[3]+y1[3]+0]);
		floatq g=floatq(data[x1[0]+y1[0]+1],data[x1[1]+y1[1]+1],data[x1[2]+y1[2]+1],data[x1[3]+y1[3]+1]);
		floatq b=floatq(data[x1[0]+y1[0]+2],data[x1[1]+y1[1]+2],data[x1[2]+y1[2]+2],data[x1[3]+y1[3]+2]);

		return Vec3q(b,g,r)*f32x4(1.0f/255.0f);
	}

}

