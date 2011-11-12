#ifndef RTRACER_VOLUME_DATA_H
#define RTRACER_VOLUME_DATA_H

#include "rtbase.h"

struct VolumeData {
	VolumeData();

	void LoadDicom(const char *folder);
	void LoadRaw(const char *folder, int width, int height, int bits);
	void Blit(gfxlib::Texture &img, int slice) const;

	vector<u16> data;
	int width, height, depth;
};


#endif

