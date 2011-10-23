#ifndef RTRACER_DICOM_H
#define RTRACER_DICOM_H

#include "rtbase.h"

struct DICOM {
	struct Slice {
		void Serialize(Serializer &sr);
	
		int width, height;
		vector<u16> data;
	};

	void LoadSlice(int y, const u16 *src);
	void Load(const char *folder);
	void Blit(gfxlib::Texture &img, int slice) const;

	vector<u16> data;
	int width, height, depth;
};


#endif

