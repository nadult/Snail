#ifndef COMPRESSION_H
#define COMPRESSION_H

#include <gfxlib_texture.h>
#include <vector>

struct CompressedPart {
	struct Info { int x, y, w, h, size; } info;
	std::vector<char> data;
};

int CompressParts(gfxlib::Texture &image, uint rank, uint nRanks, uint strapHeight,
				std::vector<CompressedPart> &parts, uint nThreads);

void DecompressParts(gfxlib::Texture &image, const std::vector<CompressedPart> &parts,
				uint nParts, uint nThreads);

#endif

