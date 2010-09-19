#ifndef COMPRESSION_H
#define COMPRESSION_H

#include <gfxlib_texture.h>
#include <vector>

struct CompressedPart {
	struct Info { int x, y, w, h, size; } info;
	std::vector<char> data;
};

void TransformData(unsigned char *ptr, uint w, uint h, uint pitch);
void ITransformData(unsigned char *ptr, uint w, uint h, uint pitch);

void CompressParts(gfxlib::Texture &image, const std::vector<int> &coords,
				std::vector<CompressedPart> &parts, uint nThreads);

struct DecompressBuffer {
	int comprSize;		// if < 0 then data is not compressed
	std::vector<unsigned char> data;
	std::vector<unsigned char> comprData;
};

// returns number of decompressed pixels
void DecompressParts(gfxlib::Texture &image, std::vector<DecompressBuffer> &parts,
				uint nParts, uint nThreads);

#endif

