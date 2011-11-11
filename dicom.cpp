#include "dicom.h"
#include <gfxlib_texture.h>
#include <algorithm>
#include <cstring>

void DICOM::Slice::Serialize(Serializer &sr) {
	char zeros[128] = {0,};
	sr.Data(zeros, sizeof(zeros));

	sr.Signature("DICM", 4);
	InputAssert(sr.IsLoading());

	int samples = 1;
	int bits = 8;
	width = height = 0;
	data.clear();

	while(sr.Pos() < sr.Size()) {
		u16 type, subType;
		u16 format, size16;
		u32 size;

		sr(type, subType, format, size16);
		if(size16 == 0 && (type != 0x08 && type != 0x20))
			sr & size;
		else
			size = size16;

		u32 end = sr.Pos() + size;
	//	printf("Chunk %x %x\n", type, subType);

		if(type == 0x28) {
			u16 tmp; sr & tmp;
			if(subType == 0x10)
				width = tmp;
			else if(subType == 0x11)
				height = tmp;
			else if(subType == 0x08)
				samples = tmp;
			else if(subType == 0x100)
				bits = tmp;
		}
		else if(type == 0x7fe0 && subType == 0x10) {
			data.resize(width * height);
			sr.Data(&data[0], size);
		}

		sr.Seek(end);
	}

	InputAssert((bits == 16 || bits == 8) && samples == 1);

	if(bits == 8) {
		u8 *src = (u8*)&data[0];
		u16 *dst = &data[0];

//		for(int n = width * height - 1; n >= 0; n--)
//			dst[n] = ((int)src[n]) * 256;
		bits = 16;
	}

	InputAssert(data.size() * sizeof(data[0]) == width * height * (bits / 8));
}

void DICOM::LoadSlice(int y, const u16 *src) {
	for(int z = 0; z < depth; z++)
		memcpy(&data[(z * height + y) * width], &src[z * width], width * sizeof(data[0]));
}

void DICOM::Load(const char *folder) {
	vector<string> files = FindFiles(folder, ".dcm", false);
	InputAssert(files.size());

	std::sort(files.begin(), files.end());

	Slice slice; Loader(files[0]) & slice;
	width = slice.width;
	height = files.size();
	depth = slice.height;

	data.resize(width * height * depth);
	LoadSlice(0, &slice.data[0]);

	printf("Loading %d slices (%dx%dx%d): ", files.size(), width, height, depth);
	fflush(stdout);

	for(int n = 1; n < files.size(); n++) {
		try { Loader(files[n]) & slice; }
		catch(...) { slice.data.resize(width * height * depth, 0); }

		InputAssert(width == slice.width);
		InputAssert(depth == slice.height);
		LoadSlice(n, &slice.data[0]);
		printf("."); fflush(stdout);
	}
	printf("\n");
}

void DICOM::Blit(gfxlib::Texture &img, int slice) const {
	int w = Min(width, img.Width()), h = Min(height, img.Height());
	InputAssert(img.GetFormat() == gfxlib::TI_R8G8B8);

	for(int y = 0; y < h; y++) {
		u8 *dst = (u8*)img.DataPointer(0);
		const u16 *src = &data[slice * width * height + y * width];
		dst += img.Pitch() * y;

		for(int x = 0; x < w; x++) {
			int value = (src[x] >> 7) * 2;
			value = Min(value, 255);

			dst[x * 3 + 0] = value;
			dst[x * 3 + 1] = value;
			dst[x * 3 + 2] = value;
		}
	}
}

