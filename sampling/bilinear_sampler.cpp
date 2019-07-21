#include "sampling/bilinear_sampler.h"

namespace sampling {

	BilinearSampler::BilinearSampler(const MipmapTexture &t) :tex(t) {
		if(tex.Width()&(tex.Width()-1)||tex.Height()&(tex.Height()-1))
			FATAL("Texture width & height must be a power of 2");
		if(tex.GetFormat() != fwk::TextureFormatId::rgb)
			FATAL("For now only R8G8B8 textures are supported");

		wMask=tex.Width()-1;
		hMask=tex.Height()-1;
		wMul=tex.Width(); hMul=tex.Height();
	}

	Vec3f BilinearSampler::operator()(const Vec2f &uv) const {
		Vec2f pos=uv*Vec2f(wMul,hMul);
		int x1(pos.x),y1(pos.y);
		int x2=x1+1,y2=y1+1;

		float dx=pos.x-x1,dy=pos.y-y1;

		const u8 *data=(u8*)tex.DataPointer();
		int pitch=tex.Pitch();

		x1&=wMask; y1&=hMask;
		x2&=wMask; y2&=hMask;
		x1*=3; x2*=3;
		y1*=pitch; y2*=pitch;

		const u8 *p[4]={data+x1+y1,data+x2+y1,data+x1+y2,data+x2+y2};

		Vec3f c[4]={
			Vec3f(p[0][2],p[0][1],p[0][0]),Vec3f(p[1][2],p[1][1],p[1][0]),
			Vec3f(p[2][2],p[2][1],p[2][0]),Vec3f(p[3][2],p[3][1],p[3][0]) };

		return Lerp(Lerp(c[0],c[1],dx),Lerp(c[2],c[3],dx),dy)*(1.0f/255.0f);
	}

	Vec3q BilinearSampler::operator()(const Vec2q &uv) const {
		Vec2q pos=uv*Vec2q(wMul,hMul);

		i32x4 x1(pos.x),y1(pos.y);
		i32x4 x2=x1+i32x4(1),y2=y1+i32x4(1);

		floatq dx=pos.x-floatq(x1),dy=pos.y-floatq(y1);

		const u8 *data=(u8*)tex.DataPointer();
		int pitch=tex.Pitch();

		x1&=i32x4(wMask); y1&=i32x4(hMask);
		x2&=i32x4(wMask); y2&=i32x4(hMask);
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

		return Vec3q(b[0],g[0],r[0])*f32x4(1.0f/255.0f);
	}

}

