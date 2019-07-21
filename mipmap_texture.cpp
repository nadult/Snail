#include "mipmap_texture.h"
#include <memory.h>
#include <fwk/gfx/texture.h>

namespace
{
	size_t Log2(size_t v) {
		size_t out = 0;

		while(v >>= 1) {
			out++;
		}
		return out;
	}

	template <class T> void FastSwap(T &tA, T &tB) {
		struct U {
			char t[sizeof(T)];
		};
		U tmp, &a = *(U *)&tA, &b = *(U *)&tB;
		tmp = a;
		a   = b;
		b   = tmp;
	}
}

MipmapTexture::MipmapTexture(const fwk::Texture &tex, fwk::TextureFormat fmt, bool hasMips) {
	if(fmt == fwk::TextureFormatId::rgba) {
		Set(tex.width(), tex.height(), fmt, hasMips? 32 : 0);
		memcpy(DataPointer(), tex.data(), tex.width() * tex.height() * 4);	
	}
	else if(fmt == fwk::TextureFormatId::rgb) {
		Set(tex.width(), tex.height(), fmt, hasMips? 32 : 0);
		for(int y = 0; y < tex.height(); y++) {
			unsigned char *dst = DataPointer() + Pitch() * y;
			for(int x = 0; x < tex.width(); x++) {
				auto color = tex(x, y);
				dst[x * 3 + 0] = color.r;
				dst[x * 3 + 1] = color.g;
				dst[x * 3 + 2] = color.b;
			}
		}
	}
	else {
		FATAL("Format %s not supported", toString(fmt.id()));
	}

	if(hasMips)
		GenMips();
}

MipmapTexture::MipmapTexture(size_t w, size_t h, fwk::TextureFormat fmt, size_t m) {
	Set(w, h, fmt, m);
}

MipmapTexture::MipmapTexture() : width(0), height(0), mips(0) {
	UpdatePtrs();
}

MipmapTexture::MipmapTexture(const MipmapTexture &rhs) : data(rhs.data), format(rhs.format) {
	width  = rhs.width;
	height = rhs.height;
	mips   = rhs.mips;
	pitch0 = rhs.pitch0;
	UpdatePtrs();
}

const MipmapTexture &MipmapTexture::operator=(const MipmapTexture &rhs) {
	if(this == &rhs)
		return *this;

	data   = rhs.data;
	width  = rhs.width;
	height = rhs.height;
	mips   = rhs.mips;
	pitch0 = rhs.pitch0;
	format = rhs.format;
	UpdatePtrs();

	return *this;
}
	
MipmapTexture::operator fwk::Texture() const {
	fwk::Texture out({(int)width, (int)height});

	if(format == fwk::TextureFormatId::rgb) {
		for(int y = 0; y < height; y++) {
			const unsigned char *src = DataPointer() + Pitch() * y;
			for(int x = 0; x < width; x++)
				out(x, y) = fwk::IColor(src[x * 3 + 0], src[x * 3 + 1], src[x * 3 + 2]);
		}
	}
	else if(format == fwk::TextureFormatId::rgba) {
	}
	else {
		FATAL("Unsupported conversion: MipmapTexture(%s) -> fwk::Texture", toString(format.id()));
	}

	return out;
}

void MipmapTexture::Set(size_t w, size_t h, fwk::TextureFormat fmt, size_t m) {
	width  = w;
	height = h;
	size_t tMaxMips = std::min(size_t(maxMips), Log2(std::max(width, height)) + 1);
	mips   = std::min(m, tMaxMips);
	format = fmt;

	if(width != 0 && height != 0) {
		if(mips == 0) mips = tMaxMips;
		size_t size = 0;
		for(m = 0; m < mips; m++) size += Size(m);
		data.resize(size);
	}
	else {
		mips   = 0;
		width  = 0;
		height = 0;
		data.clear();
		format = {};
	}

	UpdatePtrs();
	pitch0 = Pitch(0);
}

void MipmapTexture::ReallocMips(size_t mips) {
	if(data.size()) {
		MipmapTexture newSurf(width, height, format, mips);

		for(size_t m = 0, num = std::min(Mips(), newSurf.Mips()); m < num; m++)
			memcpy(newSurf.DataPointer(m), DataPointer(m), Size(m));
		Swap(newSurf);
	}
}
	
void MipmapTexture::Swap(MipmapTexture &tex) {
	std::swap(format, tex.format);
	std::swap(width, tex.width);
	std::swap(height, tex.height);
	std::swap(mips, tex.mips);
	std::swap(pitch0, tex.pitch0);
	for(size_t n = 0; n < sizeof(mipPtrs) / sizeof(mipPtrs[0]); n++)
		std::swap(mipPtrs[n], tex.mipPtrs[n]);
	data.swap(tex.data);
}

void MipmapTexture::Free() {
	width  = height = mips = 0;
	format = {};
	data.clear();
	UpdatePtrs();
}

// Complexity: O ( iters * w * h )
// TODO: Optimize this to ( w * h + iters * borderPixels )
//       (if you have nothing better to do)
void MipmapTexture::RepairAlphaBorders(size_t iters, bool allMips) {
	if(format.id() == fwk::TextureFormatId::rgba) {
		size_t startMip = 0, endMip = allMips ? 0 : mips - 1;

		//		printf("Repairing...\n");
		for(size_t mip = startMip; mip <= endMip; mip++) {
			size_t pitch = Pitch(mip);

			u8 next = 1, last = 2;

			struct Col {
				u8 r, g, b, a;
			};

			for(size_t n = 0; n < iters; n++) {
				for(size_t y = 0; y < height; y++) {
					Col *mid = (Col *)(((u8 *)DataPointer(mip)) + y * pitch);
					Col *up  = (Col *)(((u8 *)DataPointer(mip)) + (y == 0 ? height - 1 : y - 1) * pitch);
					Col *dn  = (Col *)(((u8 *)DataPointer(mip)) + (y == height - 1 ? 0 : y + 1) * pitch);

					for(size_t x = 0; x < width; x++) {
						if(mid[x].a > 0) continue;

						size_t l = (x == 0 ? width - 1 : x - 1);
						size_t p = (x == width - 1 ? 0 : x + 1);

						float count = 0.0f;
						float r     = 0, g = 0, b = 0;

						Col *neigh[8] = { up + l, up + x, up + p, dn + l, dn + x, dn + p, mid + l, mid + p };
						for(size_t n = 0; n < 8; n++) {
							u8 a = neigh[n]->a;
							if(a > 2 || a == last) {
								count++;
								r += neigh[n]->r;
								g += neigh[n]->g;
								b += neigh[n]->b;
							}
						}
						if(count > 0.0f) {
							count    = 1.0f / count;
							mid[x].r = u8(r * count);
							mid[x].g = u8(g * count);
							mid[x].b = u8(b * count);
							mid[x].a = next;
						}
					}
				}
				std::swap(next, last);
			}
			for(size_t y = 0; y < height; y++) {
				Col *mid = (Col *)(((u8 *)DataPointer(mip)) + y * pitch);
				for(size_t x = 0; x < width; x++)
					if(mid[x].a <= 2) mid[x].a = 0;
			}
		}
	}
}

void MipmapTexture::GenMip(size_t mip) {
	if(mip < 1 || mip >= Mips()) return;

	size_t srcW     = Width(mip - 1), srcH = Height(mip - 1), dstW = Width(mip), dstH = Height(mip);
	size_t srcPitch = Pitch(mip - 1), dstPitch = Pitch(mip);

	switch(format.id()) {
	case fwk::TextureFormatId::rgba:
	{
		u8 *src = (u8 *)DataPointer(mip - 1), *dst = (u8 *)DataPointer(mip);

		if(srcH == dstH) { // == 1
			for(size_t x = 0; x < dstW; x++) {
				for(int i = 0; i < 4; i++) dst[i] = (unsigned(src[i]) + unsigned(src[4 + i])) / 2;
				dst += 4;
				src += 8;
			}
		}
		else if(srcW == dstW) { // == 1
			for(size_t y = 0; y < dstH; y++) {
				for(int i = 0; i < 4; i++) dst[i] = (unsigned(src[i]) + unsigned(src[i + srcPitch])) / 2;
				src += srcPitch * 2;
				dst += dstPitch;
			}
		}
		else for(size_t y = 0; y < dstH; y++) {
				u8 *s = src + y * 2 * srcPitch, *d = dst + y * dstPitch;
				for(size_t x = 0; x < dstW; x++) {
					for(int i = 0; i < 4; i++)
						d[i] = (unsigned(s[i]) + unsigned(s[4 + i] +
									unsigned(s[i + srcPitch]) + unsigned(s[i + 4 + srcPitch]))) / 4;
					d += 4;
					s += 8;
				}
			}
		break;
	}

	case fwk::TextureFormatId::rgb:
	{
		u8 *src = (u8 *)DataPointer(mip - 1), *dst = (u8 *)DataPointer(mip);

		if(srcH == dstH) { // == 1
			for(size_t x = 0; x < dstW; x++) {
				for(int i = 0; i < 3; i++) dst[i] = (unsigned(src[i]) + unsigned(src[4 + i])) / 2;
				dst += 3;
				src += 6;
			}
		}
		else if(srcW == dstW) { // == 1
			for(size_t y = 0; y < dstH; y++) {
				for(int i = 0; i < 3; i++) dst[i] = (unsigned(src[i]) + unsigned(src[i + srcPitch])) / 2;
				src += srcPitch * 2;
				dst += dstPitch;
			}
		}
		else for(size_t y = 0; y < dstH; y++) {
				u8 *s = src + y * 2 * srcPitch, *d = dst + y * dstPitch;
				for(size_t x = 0; x < dstW; x++) {
					for(int i = 0; i < 3; i++)
						d[i] = (unsigned(s[i]) + unsigned(s[3 + i] + unsigned(s[i + srcPitch]) +
									unsigned(s[i + 3 + srcPitch]))) / 4;
					d += 3;
					s += 6;
				}
			}
		break;
	}

	default:
		FATAL("Mipmap generation for texture in %s format not supported", toString(format.id()));
		break;
	}
}

void MipmapTexture::GenMips(size_t start) {
	for(size_t n = start; n < mips; n++)
		GenMip(n);
}

void MipmapTexture::UpdatePtrs() {
	if(data.size()) {
		mipPtrs[0] = &data[0];
		for(size_t n = 1; n < mips; n++)
			mipPtrs[n] = mipPtrs[n - 1] + Size(n - 1);
		for(size_t n = mips; n < maxMips; n++)
			mipPtrs[n] = 0;
	}
	else for(size_t n = 0; n < maxMips; n++)
		mipPtrs[n] = 0;
}

unsigned MipmapTexture::loadingSkipMips           = 0;
bool	 MipmapTexture::loadingGenMips            = 0;
bool	 MipmapTexture::loadingRepairAlphaBorders = 0;
