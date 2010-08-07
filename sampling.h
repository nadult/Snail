#ifndef RTRACER_SAMPLING_H
#define RTRACER_SAMPLING_H

#include "shading.h"
#include <gfxlib_texture.h>

namespace sampling {

	struct DXTBlock { u16 c1,c2; union { u8 bits8[4]; u32 bits32; }; };

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

	struct Cache {
		DXTCache dxtCache;
	};

	class Sampler: public RefCounter {
	public:
		enum { blockSize=shading::blockSize };

		virtual ~Sampler() { }

		virtual void Sample(shading::Sample *samples,Cache&,bool mipmapping = 1) const = 0;
	};

	typedef Ptr<gfxlib::Texture> PTexture;
	typedef std::map<string, PTexture> TexDict;
	typedef Ptr<Sampler> PSampler;

	Sampler *NewSampler(PTexture);

}

#endif

