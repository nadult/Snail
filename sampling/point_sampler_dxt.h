#ifndef RTRACER_SAMPLING_POINT_SAMPLER_DXT_H
#define RTRACER_SAMPLING_POINT_SAMPLER_DXT_H

#include "rtbase.h"
#include <gfxlib_texture.h>

namespace sampling {

	class Cache;
	struct DXTBlock { u16 c1,c2; u32 bits; };

	class DXTCache {
	public:
		enum { size=16, sizeMask=size-1 };

		DXTCache();

		typedef const void* Ident;


		INLINE uint Hash(int bx,int by,int mip) const
			{ return (bx+by)&sizeMask; }
		INLINE Ident GetIdent(int bx,int by,int bShift,const DXTBlock *blocks) const
			{ return blocks+(bx<<1)+(by<<(1+bShift)); }

		void Insert(uint hash,Ident ident,int bShift);
	
		i32x4 colors[size][16];
		Ident idents[size];
	};

	class PointSamplerDXT {
	public:
		PointSamplerDXT(const gfxlib::Texture&);
		PointSamplerDXT() { }

	//	template<class Vec2>
	//	Vec3<typename Vec2::TScalar> operator[](const Vec2 &uv) const {
	//
	//	}

		Vec3f operator()(const Vec2f &uv) const; // biggest mipmap will be used
		Vec3q operator()(const Vec2q &uv) const; // biggest mipmap will be used
		Vec3q operator()(const Vec2q &uv,const Vec2q &diff) const;
		Vec3q operator()(const Vec2q &uv,const Vec2q &diff,Cache&) const;

	protected:
		void Init(const gfxlib::Texture&);

		gfxlib::Texture tex;
		uint wMask,hMask,mips,w,h,bShift;
		float hMul,wMul;
	};

}

#endif

