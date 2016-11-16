#include "volume_data.h"
#include <algorithm>
#include <cstring>
#include "fwk_gfx.h"

using fwk::Loader;

namespace
{

	struct Slice {
		void load(Stream&);
	
		int width, height;
		vector<u16> data;
	};

	void Slice::load(Stream &sr) {
		char zeros[128] = {0,};
		sr.loadData(zeros, sizeof(zeros));

		sr.signature("DICM", 4);
		ASSERT(sr.isLoading());

		int samples = 1;
		int bits = 8;
		width = height = 0;
		data.clear();

		while(sr.pos() < sr.size()) {
			u16 type, subType;
			u16 format, size16;
			u32 size;

			sr.unpack(type, subType, format, size16);
			if(size16 == 0 && (type != 0x08 && type != 0x20))
				sr >> size;
			else
				size = size16;

			u32 end = sr.pos() + size;
		//	printf("Chunk %x %x\n", type, subType);

			if(type == 0x28) {
				u16 tmp; sr >> tmp;
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
				sr.loadData(&data[0], size);
			}

			sr.seek(end);
		}

		ASSERT((bits == 16 || bits == 8) && samples == 1);

		if(bits == 8) {
			u8 *src = (u8*)&data[0];
			u16 *dst = &data[0];

	//		for(int n = width * height - 1; n >= 0; n--)
	//			dst[n] = ((int)src[n]) * 256;
			bits = 16;
		}

		ASSERT(data.size() * sizeof(data[0]) == width * height * (bits / 8));
	}

}

void VolumeData::LoadDicom(const char *folder) {
	vector<string> files = fwk::findFiles(folder, ".dcm");
	ASSERT(files.size());

	std::sort(files.begin(), files.end());

	Slice slice; Loader(files[0]) >> slice;
	width = slice.width;
	height = files.size();
	depth = slice.height;

	data.resize(width * height * depth);
	for(int z = 0; z < depth; z++)
		memcpy(&data[(z * height + 0) * width], &slice.data[z * width], width * sizeof(data[0]));

	printf("Loading %d slices (%dx%dx%d): ", (int)files.size(), width, height, depth);
	fflush(stdout);

	for(int n = 1; n < files.size(); n++) {
		try { Loader(files[n]) >> slice; }
		catch(...) { slice.data.resize(width * height * depth, 0); }

		ASSERT(width == slice.width);
		ASSERT(depth == slice.height);
		for(int z = 0; z < depth; z++)
			memcpy(&data[(z * height + n) * width], &slice.data[z * width], width * sizeof(data[0]));
		printf("."); fflush(stdout);
	}
	printf("\n");
}

void VolumeData::LoadRaw(const char *folder, int w, int h, int bits) {
	vector<string> files = fwk::findFiles(folder, "");
	data.clear();
	width = depth = height = 0;

	if(!files.size())
		return;

	std::sort(files.begin(), files.end(),
			[](const string &a, const string &b) { return a.size() == b.size()? a < b : a.size() < b.size(); } );

	ASSERT(bits == 8 || bits == 16);
	
	width = w;
	height = h;
	depth = files.size();
	data.resize(width * height * depth);

	u16 min = 0xffff, max = 0;

	for(int n = 0; n < depth; n++) {
		u16 *dst = &data[n * width * height];

		if(bits == 16) {
			Loader(files[n]).loadData(dst, width * height * 2);
		}
		else {
			vector<u8> temp(width, height);
			Loader(files[n]).loadData(&temp[0], width * height);
			for(int i = 0, count = width * height; i < count; i++)
				dst[i] = ((int)temp[i]) * 256;
		}

		for(int i = 0, count = width * height; i < count; i++) {
			min = Min(min, dst[i]);
			max = Max(max, dst[i]);
		}
	}

	printf("Min / Max: %d %d\n", (int)min, (int)max);
}

void VolumeData::Blit(fwk::Texture &img, int slice) const {
	int w = Min(width, img.width()), h = Min(height, img.height());
	//TODO: testme

	for(int y = 0; y < h; y++) {
		const u16 *src = &data[slice * width * height + y * width];

		for(int x = 0; x < w; x++) {
			int value = (src[x] >> 7) * 2;
			value = Min(value, 255);
			img(x, y) = {value, value, value};
		}
	}
}
