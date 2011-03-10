#include "compression.h"
#include "thread_pool.h"
#include "rtbase.h"
#include "quicklz/quicklz.h"
#include "quicklz/quicklz.c"

static void Decompress(unsigned char *data) {
	enum {
		width = 32,
		height = 32,
	};

	return;
	for(int y = 0; y < height; y++) {
		unsigned int bits[8];
		__builtin_memcpy(bits, data + y * width, 32);

		for(int x = 0; x < width; x++) {
			unsigned char c = 0;
			c |= bits[0] & (1u << x)? 1   : 0;
			c |= bits[1] & (1u << x)? 2   : 0;
			c |= bits[2] & (1u << x)? 4   : 0;
			c |= bits[3] & (1u << x)? 8   : 0;
			c |= bits[4] & (1u << x)? 16  : 0;
			c |= bits[5] & (1u << x)? 32  : 0;
			c |= bits[6] & (1u << x)? 64  : 0;
			c |= bits[7] & (1u << x)? 128 : 0;
			data[x + y * width] = c;
		}
	}
	return;
}

#define COMPRESSION_ENABLED
//#define DWT_ENABLED

#if QLZ_STREAMING_BUFFER > 0
#define PROGRESSIVE
#endif

namespace {

	struct DecompressTask: public thread_pool::Task {
		DecompressTask() { }
		DecompressTask(gfxlib::Texture &image, DecompressBuffer *buffer)
			:image(&image), buf(buffer) { }
#if defined(__PPC) || defined(__PPC__)
		void Work(SPEContext*) {
#else
		void Work() {
#endif
			if(buf->comprSize < 0) {
				int *idata = (int*)&buf->comprData[0];
				int nParts = idata[0];
				unsigned char *ptr = &buf->comprData[nParts * 16 + 16];

				for(int n = 0; n < nParts; n++) {
					int sx = idata[n * 4 + 1], sy = idata[n * 4 + 2];
					int w  = idata[n * 4 + 3], h = idata[n * 4 + 4];

					unsigned char *dst =
						(unsigned char*)image->DataPointer() + sx * 3 + sy * image->Pitch();
					unsigned char *srcr = ptr + w * h * 0;
					unsigned char *srcg = ptr + w * h * 1;
					unsigned char *srcb = ptr + w * h * 2;

					for(int y = 0; y < h; y++) {
						unsigned char *__restrict__ tdst = dst + y * image->Pitch();
						unsigned char *tsrcr = srcr + y * w;
						unsigned char *tsrcg = srcg + y * w;
						unsigned char *tsrcb = srcb + y * w;

						for(int x = 0; x < w; x++) {
							tdst[0] = tsrcr[x];
							tdst[1] = tsrcg[x];
							tdst[2] = tsrcb[x];
							tdst += 3;
						}
					}

					ptr += w * h * 3;
				}
				return;
			}

			int size = qlz_size_decompressed((char*)&buf->comprData[0]);

			if(buf->data.size() < size)
				buf->data.resize(size);
			char scratch[QLZ_SCRATCH_DECOMPRESS];

#ifdef PROGRESSIVE
			memset(scratch, 0, sizeof(scratch));
			int doffset = qlz_decompress((char*)&buf->comprData[0], (char*)&buf->data[0], scratch);
			int *idata = (int*)&buf->data[0];
			int nParts = idata[0];
			int coffset = qlz_size_compressed((char*)&buf->comprData[0]);
			
			size += nParts * 32 * 32 * 3;
			if(buf->data.size() < size)
				buf->data.resize(size);

			for(int n = 0; n < nParts; n++) {
				doffset += qlz_decompress((char*)&buf->comprData[coffset], (char*)&buf->data[doffset],
						scratch);
				coffset += qlz_size_compressed((char*)&buf->comprData[coffset]);
			}
#else
			qlz_decompress((char*)&buf->comprData[0], (char*)&buf->data[0], scratch);
			int *idata = (int*)&buf->data[0];
			int nParts = idata[0];
#endif

			unsigned char *ptr = &buf->data[nParts * 16 + 16];
			for(int n = 0; n < nParts; n++) {
				int sx = idata[n * 4 + 1], sy = idata[n * 4 + 2];
				int w  = idata[n * 4 + 3], h = idata[n * 4 + 4];

				unsigned char *dst =
					(unsigned char*)image->DataPointer() + sx * 3 + sy * image->Pitch();
				unsigned char *srcr = ptr + w * h * 0;
				unsigned char *srcg = ptr + w * h * 1;
				unsigned char *srcb = ptr + w * h * 2;

				Decompress(srcr);
				Decompress(srcg);
				Decompress(srcb);

				for(int y = 0; y < h; y++) {
					unsigned char *__restrict__ tdst = dst + y * image->Pitch();
					unsigned char *tsrcr = srcr + y * w;
					unsigned char *tsrcg = srcg + y * w;
					unsigned char *tsrcb = srcb + y * w;

					for(int x = 0; x < w; x++) {
						unsigned char red = tsrcr[x];
						tdst[2] = red;
						tdst[1] = tsrcg[x] + red;
						tdst[0] = tsrcb[x] + red;
						tdst += 3;
					}
				}

				ptr += w * h * 3;
			}
		}

		gfxlib::Texture *image;
		DecompressBuffer *buf;
	};

}

void DecompressParts(gfxlib::Texture &image, vector<DecompressBuffer> &parts, uint nParts,
		uint nThreads) {
	InputAssert(parts.size() >= nParts);
	InputAssert(image.GetFormat().BytesPerPixel() == 3);

	vector<DecompressTask> tasks(nParts);
	for(size_t n = 0; n < nParts; n++)
		tasks[n] = DecompressTask(image, &parts[n]);
	thread_pool::Run(tasks, nThreads);
}

