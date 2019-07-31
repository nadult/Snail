#pragma once

#include "rtbase.h"

namespace fwk { class Texture; }

struct VolumeData {
	void LoadDicom(const char *folder);
	void LoadRaw(const char *folder, int width, int height, int bits);
	void Blit(fwk::Texture &img, int slice) const;

	vector<u16> data;
	int width = 0, height = 0, depth = 0;
};
