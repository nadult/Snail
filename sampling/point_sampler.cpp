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
		halfInvW=0.5f/float(tex.Width()); halfInvH=0.5f/float(tex.Height());
	}

	template <class Int,class Vec2>
	INLINE Vec2 ClampTexCoord(const Vec2 &coord) {
		Vec2 uv=coord-Vec2(Int(coord.x),Int(coord.y));
		uv.x=Condition(uv.x<0.0f,uv.x+1.0f,uv.x);
		uv.y=Condition(uv.y<0.0f,uv.y+1.0f,uv.y);
		return uv;
	}

	Vec3f PointSampler::operator()(const Vec2f &coord) const {
		Vec2f uv=ClampTexCoord<int>(coord+Vec2f(halfInvW,halfInvH));
		Vec2f pos=uv*Vec2f(wMul,hMul);
		int x1(pos.x),y1(pos.y);

		const u8 *data=(u8*)tex.DataPointer();
		int pitch=tex.Pitch();

		assert(x1<w&&y1<h);
	//	x1&=wMask; y1&=hMask;
		x1*=3;
		y1*=pitch;

		return Vec3f(data[x1+2+y1],data[x1+1+y1],data[x1+0+y1])*(1.0f/255.0f);
	}

	Vec3q PointSampler::operator()(const Vec2q &coord) const {
		Vec2q uv=ClampTexCoord<i32x4>(coord+Vec2q(halfInvW,halfInvH));
		Vec2q pos=uv*Vec2q(wMul,hMul);

		i32x4 x1(pos.x),y1(pos.y);

		const u8 *data=(u8*)tex.DataPointer();
		int pitch=tex.Pitch();

		assert(ForAll(x1<i32x4(w)&&y1<i32x4(h)));
	//	x1&=i32x4(wMask); y1&=i32x4(hMask);
		x1=x1+x1+x1;

		y1[0]*=pitch; y1[1]*=pitch; y1[2]*=pitch; y1[3]*=pitch;

		i32x4 o=x1+y1;

#define R(b) data[o[b]+0]
#define G(b) data[o[b]+1]
#define B(b) data[o[b]+2]

		floatq r=floatq(R(0),R(1),R(2),R(3));
		floatq g=floatq(G(0),G(1),G(2),G(3));
		floatq b=floatq(B(0),B(1),B(2),B(3));

#undef R
#undef G
#undef B

		return Vec3q(b,g,r)*f32x4(1.0f/255.0f);
	}

}

