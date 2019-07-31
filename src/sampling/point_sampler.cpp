#include "sampling/point_sampler.h"
#include "mipmap_texture.h"

namespace sampling {

	PointSampler::PointSampler(const PMipmapTexture tex) : tex(tex) {
		if(tex->Width() & (tex->Width() - 1) || tex->Height() & (tex->Height() - 1))
			FATAL("Texture width & height must be a power of 2");
		if(tex->GetFormat().id() != fwk::TextureFormatId::rgb)
			FATAL("For now only R8G8B8 textures are supported");
		if(tex->Width() > 65536 || tex->Height() > 65536)
			FATAL("Maximum width / height of a texture is: 65536");
//		if(tex->Mips()>1&&tex->Width()!=tex->Height())
//			FATAL("Mipmapped textures must have width same as height");
		Update();
	}

	PointSampler::PointSampler(const PointSampler &rhs) :tex(rhs.tex) {
		Update();
	}

	void PointSampler::operator=(const PointSampler &rhs) {
		tex = rhs.tex;
		Update();
	}

	void PointSampler::Update() {
		wMask  = tex->Width() - 1;
		hMask  = tex->Height() - 1;
		wMul   = tex->Width() - 1;
		hMul   = tex->Height() - 1;
		mips   = tex->Mips();
		w      = tex->Width();
		h      = tex->Height();
		wShift = 1;
		while((1 << wShift) < w)
			wShift++;

		for(int n = 0; n < 16; n++)
			mipPitch[n] = tex->Pitch(n);
	}

	template <class Int, class Vec2>
	INLINE Vec2 ClampTexCoord(const Vec2 &coord) {
		Vec2 uv = coord - Vec2(Int(coord.x), Int(coord.y));

		using Scalar = typename Vec2::TScalar;
		uv.x = Condition(uv.x < 0.0f, uv.x + Scalar(1.0f), uv.x);
		uv.y = Condition(uv.y < 0.0f, uv.y + Scalar(1.0f), uv.y);
		return uv;
	}

	Vec3f PointSampler::operator()(const Vec2f &coord) const {
		Vec2f pos = coord * Vec2f(wMul, hMul);
		int   x1(pos.x), y1(pos.y);

		const u8 *data = (u8 *)tex->DataPointer();
		int pitch = mipPitch[0];

		x1 &= wMask;
		y1 &= hMask; // in case of NANs etc
		x1 *= 3;
		y1 *= pitch;

		return Vec3f(data[x1 + 2 + y1], data[x1 + 1 + y1], data[x1 + 0 + y1]) * (1.0f / 255.0f);
	}

	Vec3q PointSampler::operator()(const Vec2q &coord) const {
		Vec2q pos = coord * Vec2q(wMul, hMul);
		i32x4 x1(pos.x), y1(pos.y);

		x1 &= i32x4(wMask);
		y1 &= i32x4(hMask);
		x1  = x1 + x1 + x1;
			
		//TODO: wylaczyc odbicie w pionie
		y1 = int(tex->Height()) -1 - y1;

		const u8 *data = (u8 *)tex->DataPointer();
		int      pitch = mipPitch[0];

		y1 *= pitch;

		floatq r = floatq(data[x1[0] + y1[0] + 0], data[x1[1] + y1[1] + 0], data[x1[2] + y1[2] + 0], data[x1[3] + y1[3] + 0]);
		floatq g = floatq(data[x1[0] + y1[0] + 1], data[x1[1] + y1[1] + 1], data[x1[2] + y1[2] + 1], data[x1[3] + y1[3] + 1]);
		floatq b = floatq(data[x1[0] + y1[0] + 2], data[x1[1] + y1[1] + 2], data[x1[2] + y1[2] + 2], data[x1[3] + y1[3] + 2]);

		return Vec3q(r, g, b) * f32x4(1.0f / 255.0f);
	}

	Vec3q PointSampler::operator()(const Vec2q &coord, const Vec2q &diff) const {
		Vec2q pos = (coord) * Vec2q(wMul, hMul);
		i32x4 x(pos.x), y(pos.y);

		uint mip = 0;
		{
			floatq min    = Min(diff.x * wMul, diff.y * hMul);
			uint   pixels = uint(Minimize(min));
			mip = 0;
			while(pixels) {
				mip++;
				pixels >>= 1;
			}
			mip = Min(mip, tex->Mips() - 1);
		}

		const u8 *data = (u8 *)tex->DataPointer(mip);
		int pitch = mipPitch[mip];

		x >>= mip;
		y >>= mip;
		x  &= i32x4(wMask >> mip);
		y  &= i32x4(hMask >> mip);

		x = x + x + x;
		y *= pitch;

		floatq r = floatq(data[x[0] + y[0] + 0], data[x[1] + y[1] + 0], data[x[2] + y[2] + 0], data[x[3] + y[3] + 0]);
		floatq g = floatq(data[x[0] + y[0] + 1], data[x[1] + y[1] + 1], data[x[2] + y[2] + 1], data[x[3] + y[3] + 1]);
		floatq b = floatq(data[x[0] + y[0] + 2], data[x[1] + y[1] + 2], data[x[2] + y[2] + 2], data[x[3] + y[3] + 2]);

		return Vec3q(r, g, b) * f32x4(1.0f / 255.0f);
	}

	// bilinear filtering
	void PointSampler::Sample(shading::Sample *samples, Cache &, bool mipmapping) const {
		for(int k = 0 + 0; k < 4; k++) {
			shading::Sample &s = samples[k];

			Vec2q pos = ClampTexCoord <i32x4>(s.texCoord) * Vec2q(wMul, hMul);

			uint mip = 0;
			if(mipmapping) {
				floatq min = Min(s.texDiff.x * wMul, s.texDiff.y * hMul);
				uint   pixels = uint(Minimize(min) * 0.6f);
				mip = 0;
				while(pixels) {
					mip++;
					pixels >>= 1;
				}
				mip = Min(mip, tex->Mips() - 1);
			}

			const u8 *data = (u8 *)tex->DataPointer(mip);
			int pitch = mipPitch[mip];

			i32x4 x1(pos.x), y1(pos.y);
			i32x4 x2 = x1 + i32x4(1), y2 = y1 + i32x4(1);

			floatq dx = pos.x - floatq(x1), dy = pos.y - floatq(y1);

			//TODO: wylaczyc odbicie w pionie
			y1 = int(tex->Height()) - y1;
			y2 = int(tex->Height()) - y2;

			x1 >>= mip;
			y1 >>= mip;
			x2 >>= mip;
			y2 >>= mip;

			x1 &= i32x4(wMask >> mip);
			y1 &= i32x4(hMask >> mip);
			x2 &= i32x4(wMask >> mip);
			y2 &= i32x4(hMask >> mip);
			x1  = x1 + x1 + x1;
			x2  = x2 + x2 + x2;

			y1 *= pitch;
			y2 *= pitch;

			i32x4 o[4] = { x1 + y1, x2 + y1, x1 + y2, x2 + y2 };

#define DATA(a, b)    *(u32*)&data[o[a][b]]

			//	s.temp1 = Vec3q(B(0, 0), G(0, 0), R(0, 0)) * f32x4(1.0f / 255.0f);
			//	continue;
			floatq red, green, blue; {
				// TODO upewnic sie ze mozna czytac zawsze 4 bajty
				i32x4 col0( DATA(0, 0), DATA(0, 1), DATA(0, 2), DATA(0, 3) );
				i32x4 col1( DATA(1, 0), DATA(1, 1), DATA(1, 2), DATA(1, 3) );
				i32x4 col2( DATA(2, 0), DATA(2, 1), DATA(2, 2), DATA(2, 3) );
				i32x4 col3( DATA(3, 0), DATA(3, 1), DATA(3, 2), DATA(3, 3) );

				floatq r[4], g[4], b[4];

				r[0] = floatq( col0 & i32x4(255) );
				g[0] = floatq( Shr<8>(col0) & i32x4(255) );
				b[0] = floatq( Shr<16>(col0) & i32x4(255) );

				r[1] = floatq( col1 & i32x4(255) );
				g[1] = floatq( Shr<8>(col1) & i32x4(255) );
				b[1] = floatq( Shr<16>(col1) & i32x4(255) );

				r[2] = floatq( col2 & i32x4(255) );
				g[2] = floatq( Shr<8>(col2) & i32x4(255) );
				b[2] = floatq( Shr<16>(col2) & i32x4(255) );

				r[3] = floatq( col3 & i32x4(255) );
				g[3] = floatq( Shr<8>(col3) & i32x4(255) );
				b[3] = floatq( Shr<16>(col3) & i32x4(255) );

				red = Lerp(Lerp(r[0], r[1], dx), Lerp(r[2], r[3], dx), dy);
				green = Lerp(Lerp(g[0], g[1], dx), Lerp(g[2], g[3], dx), dy);
				blue = Lerp(Lerp(b[0], b[1], dx), Lerp(b[2], b[3], dx), dy);
			}
#undef DATA

			s.temp1 = Vec3q(red, green, blue) * f32x4(1.0f / 255.0f);
		}
	}

/*	// filtering disabled
	void PointSampler::Sample(shading::Sample *samples, Cache&) const {
	    
		uint mip; {
	        floatq min = Min(samples[0].texDiff.x * wMul, samples[0].texDiff.y * hMul);
	        uint pixels = uint(Minimize(min));
	        mip = 0; while(pixels) { mip++; pixels>>=1; }
	        mip = Min(mip, tex->Mips()-1);
	    }
	    
		const u8 *data=(u8*)tex->DataPointer(mip);
	    int pitch=mipPitch[mip];
	    
		for(int k=0;k<4;k++) {
	        shading::Sample &s=samples[k];
	        
			Vec2q pos = (s.texCoord) * Vec2q(wMul, hMul);
	        i32x4 x(pos.x), y(pos.y);
	        
			x >>= mip; y >>= mip;
	        x &= i32x4(wMask >> mip);
	        y &= i32x4(hMask >> mip);
	        
			x = x + x + x;
			y *= pitch;
 
	        floatq r = floatq(data[x[0] + y[0] + 0], data[x[1] + y[1] + 0], data[x[2] + y[2] + 0], data[x[3] + y[3] + 0]);
	        floatq g = floatq(data[x[0] + y[0] + 1], data[x[1] + y[1] + 1], data[x[2] + y[2] + 1], data[x[3] + y[3] + 1]);
	        floatq b = floatq(data[x[0] + y[0] + 2], data[x[1] + y[1] + 2], data[x[2] + y[2] + 2], data[x[3] + y[3] + 2]);
	        
			s.temp1 = Vec3q(b, g, r) * f32x4(1.0f / 255.0f);
	    }
	} */
}
