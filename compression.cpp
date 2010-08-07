#include "compression.h"
#include "quicklz/quicklz.h"
#include "quicklz/quicklz.c"
#include "thread_pool.h"
#include "rtbase.h"


static void dwt_encode_1(short *a, int size) {
	int half = size / 2;

	for (int i = 0; i < half - 1; i++)
		a[i * 2 + 1] -= (a[i * 2] + a[i * 2 + 2]) >> 1;
	a[(half - 1) * 2 + 1] -= a[(half - 1) * 2];

	a[0] += (a[1] + 1) >> 1;
	for (int i = 1; i < half; i++)
		a[i * 2 + 0] += (a[i * 2 - 1] + a[i * 2 + 1] + 2) >> 2;
}

static void dwt_decode_1(short *a, int size) {
	int half = size / 2;

	a[0] -= (a[1] + 1) >> 1;
	for (int i = 1; i < half; i++)
		a[i * 2 + 0] -= (a[i * 2 - 1] + a[i * 2 + 1] + 2) >> 2;

	for (int i = 0; i < half - 1; i++)
		a[i * 2 + 1] += (a[i * 2] + a[i * 2 + 2]) >> 1;
	a[(half - 1) * 2 + 1] += a[(half - 1) * 2];
}

static void deinterleave(const short *src, short *dst, int tw) {
	int half = tw / 2;
	for(int x = 0; x < half; x++) {
		dst[x] = src[x * 2 + 0];
		dst[x + half] = src[x * 2 + 1];
	}
}

static void deinterleave_v(const short *src, short *dst, int th, int w) {
	int half = th / 2;
	int yh = half * w;

	for(int y = 0; y < half; y++) {
		int ty = y * w;
		dst[ty] = src[y * 2 + 0];
		dst[ty + yh] = src[y * 2 + 1];
	}
}

static void dwt_encode_v(short *lines, int w, int tw, int th) {
	short row[2048 * 4];
	for(int y = 0; y < th; y++)
		row[y] = lines[y * w];
	dwt_encode_1(row, th);
	deinterleave_v(row, lines, th, w);
}

static void dwt_transform(short *data, int w, int h) {
	for(int l = 0; l < 8; l++) {
		int tw = (w >> l) & ~1;
		int th = (h >> l) & ~1;

		if(th >= 2)
			for(int x = 0; x < tw; x++)
				dwt_encode_v(data + x, w, tw, th);
		if(tw >= 2)
			for(int y = 0; y < th; y++) {
				short line[2048 * 4];
				short *tline = data + y * w;
				for(int x = 0; x < tw; x++)
					line[x] = tline[x];
				dwt_encode_1(line, tw);
				deinterleave(line, tline, tw);
			}
	}
}


static void interleave(const short *src, short *dst, int tw) {
	int half = tw / 2;
	for(int x = 0; x < half; x++) {
		dst[x * 2 + 0] = src[x];
		dst[x * 2 + 1] = src[x + half];
	}
}

static void interleave_v(const short *src, short *dst, int th, int w) {
	int half = th / 2;
	int yh = half * w;

	for(int y = 0; y < half; y++) {
		int ty = y * w;
		dst[y * 2 + 0] = src[ty];
		dst[y * 2 + 1] = src[ty + yh];
	}
}

static void idwt_transform(short *data, int w, int h) {
	for(int l = 7; l >= 0; l--) {
		int tw = (w >> l) & ~1;
		int th = (h >> l) & ~1;

		if(tw >= 2)
			for(int y = 0; y < th; y++) {
				short line[2048 * 4];
				short *tline = data + y * w;
				interleave(tline, line, tw);
				dwt_decode_1(line, tw);
				for(int x = 0; x < tw; x++)
					tline[x] = line[x];
			}
		if(th >= 2)
			for(int x = 0; x < tw; x++) {
				short row[2048 * 4];
				short *lines = data + x;
				interleave_v(lines, row, th, w);
				dwt_decode_1(row, th);
				for(int y = 0; y < th; y++)
					lines[y * w] = row[y];
			}
	} 
}

static void filter_coeffs(short *data, int w, int h, int tmax) {
	for(int l = 0; l < 8; l++) {
		int tw = (w >> l) & ~1;
		int th = (h >> l) & ~1;
		int hw = tw >> 1;
		int hh = th >> 1;

		for(int y = 0; y < hh; y++) {
			short *line = data + y * w;
			for(int x = hw; x < tw; x++)
				if(abs(line[x]) < (tmax >> (l * 2 + 1)))
					line[x] = 0;
		}
		for(int y = hh; y < th; y++) {
			short *line = data + y * w;
			for(int x = 0; x < hw; x++)
				if(abs(line[x]) < (tmax >> (l * 2 + 1)))
					line[x] = 0;
		}
		for(int y = hh; y < th; y++) {
			short *line = data + y * w;
			for(int x = hw; x < tw; x++)
				if(abs(line[x]) < (tmax >> (l * 2)))
					line[x] = 0;
		}
	}
}

#define COMPRESSION_ENABLED
//#define SPU_COMPRESSION
//#define DWT_ENABLED

#if defined(__PPC) || defined(__PPC__)
#include "spu/context.h"
#include "spu/compression.h"
#endif



namespace {

#ifdef DWT_ENABLED
	typedef short pix;
#else
	typedef u8 pix;
#endif

	//TODO: obsluga dla kawalkow o dowolnej wielkosci i pozycji
	//TODO: pitch moze byc nierowne w * bpp
	struct CompressTask: public thread_pool::Task {
		CompressTask(gfxlib::Texture &image, CompressedPart *part)
			:image(&image), part(part) {
#if defined(SPU_COMPRESSION) && (defined(__PPC__) || defined(__PPC))
				preload = &spe_compression;
#endif
			}
#if defined(__PPC) || defined(__PPC__)
		void Work(SPEContext *context) {
#else
		void Work() {
#endif
			int bpp = image->GetFormat().BytesPerPixel();
#ifdef COMPRESSION_ENABLED
			const u8 *src = (const u8*)image->DataPointer() + part->info.y * image->Pitch();

#if defined(SPU_COMPRESSION) && (defined(__PPC) || defined(__PPC__))
			TaskInfo info ALIGN256;
			char tdata[1024 * 16 * 4 + 400] ALIGN256;
			memcpy(tdata, src, part->info.w * part->info.h * bpp);

			info.w = part->info.w; info.h = part->info.h;
			info.bpp = bpp;
			info.data = (unsigned long long)tdata;
			context->Run(&info, 0);
			memcpy(&part->data[0], tdata, info.outSize);
			part->info.size = info.outSize;
#else
			char scratch[QLZ_SCRATCH_COMPRESS];
			if(bpp == 3) {
				int w = part->info.w, h = part->info.h, nPixels = w * h;
				pix buf[w * h * 3];
				InputAssert(part->info.size == w * h * 3);
				pix *channel[3] = { buf + nPixels * 0, buf + nPixels * 1, buf + nPixels * 2 };

				for(int n = 0; n < nPixels; n++) {
					channel[0][n] = src[n * 3 + 0];
					channel[1][n] = src[n * 3 + 1] - src[n * 3 + 0];
					channel[2][n] = src[n * 3 + 2] - src[n * 3 + 0];
				}
	#ifdef DWT_ENABLED
				for(int c = 0; c < 3; c++) {
					dwt_transform(channel[c], w, h);
					filter_coeffs(channel[c], w, h, 32);	
				}
	#endif
				part->info.size = qlz_compress(buf, &part->data[0], part->info.size * sizeof(pix), scratch);
			}
			else {
				part->info.size = qlz_compress(src, &part->data[0], part->info.size, scratch);
			}
#endif
#else
			for(int y = 0; y < part->info.h; y++)
				memcpy(	&part->data[y * part->info.w * bpp],
						(u8*)image->DataPointer() + (part->info.y + y) * image->Pitch(),
						part->info.w * bpp);
			part->info.size = part->info.w * part->info.h * bpp;
#endif
		}

		const gfxlib::Texture *image;
		CompressedPart *part;
	};

	struct DecompressTask: public thread_pool::Task {
		DecompressTask() { }
		DecompressTask(gfxlib::Texture &image, const CompressedPart *part)
			:image(&image), part(part) { }
#if defined(__PPC) || defined(__PPC__)
		void Work(SPEContext*) {
#else
		void Work() {
#endif
			int bpp = image->GetFormat().BytesPerPixel();
#ifdef COMPRESSION_ENABLED
			char scratch[QLZ_SCRATCH_DECOMPRESS];
			Assert(part->info.x == 0 && part->info.w == image->Width());
			Assert(qlz_size_decompressed(&part->data[0]) == part->info.w * part->info.h * bpp);
			u8 *dst = (u8*)image->DataPointer() + part->info.y * image->Pitch();
			int w = part->info.w, h = part->info.h;

			if(bpp == 3) {
				pix buf[w * h * 3];
				int nPixels = qlz_decompress(&part->data[0], buf, scratch) / (3 * sizeof(pix));
				InputAssert(nPixels == w * h);
				pix *channel[3] = { buf + nPixels * 0, buf + nPixels * 1, buf + nPixels * 2 };

#ifdef DWT_ENABLED
				for(int c = 0; c < 3; c++)
					idwt_transform(channel[c], w, h);
				for(int n = 0; n < nPixels; n++) {
					dst[n * 3 + 0] = Clamp((int)channel[0][n], 0, 255);
					dst[n * 3 + 1] = Clamp((int)channel[1][n] + (int)channel[0][n], 0, 255);
					dst[n * 3 + 2] = Clamp((int)channel[2][n] + (int)channel[0][n], 0, 255);
				}
#else	
				for(int n = 0; n < nPixels; n++) {
					dst[n * 3 + 0] = channel[0][n];
					dst[n * 3 + 1] = channel[1][n] + channel[0][n];
					dst[n * 3 + 2] = channel[2][n] + channel[0][n];
				}
#endif
			}
			else {
				qlz_decompress(&part->data[0], dst, scratch);
			}
#else
			for(int y = 0; y < part->info.h; y++)
				memcpy(	(u8*)image->DataPointer() + (part->info.y + y) * image->Pitch(),
						&part->data[y * part->info.w * bpp], part->info.w * bpp);
#endif
		}

		gfxlib::Texture *image;
		const CompressedPart *part;
	};

}

int CompressParts(gfxlib::Texture &image, uint rank, uint nRanks, uint strapHeight,
				vector<CompressedPart> &parts, uint nThreads) {
	vector<CompressTask> tasks;
	tasks.reserve(128);

	int num = 0;
	for(uint sy = 0; sy < (image.Height() + strapHeight - 1) / strapHeight; sy++)
		if(sy % nRanks == rank)
			num++;
	if(parts.size() < num)
		parts.resize(num);
	num = 0;

	for(uint sy = 0; sy < (image.Height() + strapHeight - 1) / strapHeight; sy++)
		if(sy % nRanks == rank) {
			int height = Min(image.Height() - sy * strapHeight, strapHeight);
			int bpp = image.GetFormat().BytesPerPixel();
			CompressedPart *part = &parts[num++];
			part->info.x = 0; part->info.w = image.Width(); part->info.y = sy * strapHeight;
			part->info.h = Min(image.Height() - sy * strapHeight, strapHeight);
			part->info.size = part->info.w * part->info.h * bpp;
			if(part->data.size() < part->info.size * 2 + 400)
				part->data.resize(part->info.size * 2 + 400);
			tasks.push_back(CompressTask(image, part));
		}

	thread_pool::Run(tasks, nThreads);
	return num;
}

void DecompressParts(gfxlib::Texture &image, const vector<CompressedPart> &parts, uint nParts,
		uint nThreads) {
	Assert(parts.size() >= nParts);

//	for(int n = 0; n < nParts; n++)
//		DecompressTask(image, &parts[n]).Work();

	vector<DecompressTask> tasks(nParts);
	for(size_t n = 0; n < nParts; n++)
		tasks[n] = DecompressTask(image, &parts[n]);
	thread_pool::Run(tasks, nThreads);
}

