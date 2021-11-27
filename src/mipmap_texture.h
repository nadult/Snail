#ifndef MIPMAP_TEXTURE_H
#define MIPMAP_TEXTURE_H

#include "rtbase.h"
#include <fwk/gfx_base.h>
#include <fwk/gfx/gl_format.h>

// TODO: rename to MipmapImage
class MipmapTexture
{
public:
	enum { maxMips = 32 };
	using Format = fwk::GlFormat;

	MipmapTexture(const fwk::Image&, Format = Format::rgba8, bool hasMips = true);
	MipmapTexture(size_t width, size_t height, Format fmt, size_t mips = 0);
	MipmapTexture(const MipmapTexture &rhs);
	const MipmapTexture &operator=(const MipmapTexture &);
	MipmapTexture();

	operator fwk::Image() const;

	// poprzednie dane zostaja utracone
	void Set(size_t width, size_t height, Format fmt, size_t mips = 0);

	void Set(fwk::Image, Format);
	
	// Zmienia ilosc mipmap, parametr 0 oznacza max
	// procedura jedynie alokuje odpowiednia ilosc pamieci i kopiuje
	// to co moze, nowo utworzone mipmapy nalezy wypelnic samemu
	void ReallocMips(size_t mips);
	void Free();

	size_t Width(size_t mip = 0) const {
		return std::max(width >> mip, size_t(1));
	}
	size_t Height(size_t mip = 0) const {
		return std::max(height >> mip, size_t(1));
	}
	size_t Pitch(size_t mip = 0) const {
		return imageRowSize(format, std::max(width >> mip, size_t(1)));
	}

	// size (in bytes) of given mipmap
	size_t Size(size_t mip = 0) const {
		return imageSize(format, std::max(width >> mip, size_t(1)), std::max(height >> mip, size_t(1)));
	}

	unsigned char *DataPointer(size_t mip = 0) const {
		return mipPtrs[mip];
	}
	size_t Mips() const {
		return mips;
	}
	auto GetFormat() const {
		return format;
	}

	// Works only for 32-bit formats, and it's kinda slow.
	void RepairAlphaBorders(size_t iters = 1, bool allMips = 1);

	// Na razie jedynie 32-bitowe formaty sa obslugiwane
	void GenMip(size_t mip);

	// generuje mipmapy zaczynajac od podanej
	void GenMips(size_t startMip = 1);

	void Swap(MipmapTexture &tex);

	// Every option is disabled by default:

	// Number of mipmaps (starting from the biggest one) skipped
	static unsigned loadingSkipMips;

	// Generates all missing mipmaps down to 1x1 while loading
	static bool loadingGenMips;

	// Automaticly repairs alpha borders after load
	static bool loadingRepairAlphaBorders;

private:
	void UpdatePtrs();

	vector <unsigned char> data;
	unsigned char *mipPtrs[maxMips]; // TODO: bleee, lepiej dynamicznie
	size_t        width, height, mips, pitch0;
	Format format;
};

#endif
