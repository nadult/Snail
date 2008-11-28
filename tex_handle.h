#ifndef GFX_TEXTURE_HANDLE_H
#define GFX_TEXTURE_HANDLE_H

#include <gfxlib_texture.h>
#include "rtbase.h"

	//! Teksturka
	class TexHandle: public Resource
	{
	public:
		typedef gfxlib::TextureFormat Format;
		typedef gfxlib::Texture Surface;

		TexHandle();
		~TexHandle();

		void Serialize(Serializer &sr);

		void Create(uint mips);
		
		void SetSurface(const Surface &in);
		void GetSurface(Surface& out);
		
		void SetMip(uint mip,Format fmt,void *pixels);
		void CreateMip(uint mip,uint width,uint height,Format fmt);

		void UpdateMip(uint mip,uint x,uint y,uint w,uint h,void *pixels,uint pixelsInRow);

		void Free();
		uint Width() const;
		uint Height() const;
		inline uint Mips() const { assert(id!=~0); return mips; }
		Format GetFormat() const;

		inline uint Id() const { return id; }

	private:
		TexHandle(const TexHandle&) { }
		void operator=(const TexHandle&) { }

		uint id,mips;
	};

#endif
