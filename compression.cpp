#include "compression.h"
#include "quicklz/quicklz.h"
#include "quicklz/quicklz.c"
#include "thread_pool.h"
#include "rtbase.h"

using std::vector;

namespace {

	struct CompressTask: public thread_pool::Task {
		CompressTask(const gfxlib::Texture &image, CompressedPart *part)
			:image(&image), part(part) { }
		void Work() {
			char scratch[QLZ_SCRATCH_COMPRESS];
			const char *src = (const char*)image->DataPointer() + part->info.y * image->Pitch();
			part->info.size = qlz_compress(src, &part->data[0], part->info.size, scratch);
		}

		const gfxlib::Texture *image;
		CompressedPart *part;
	};

	struct DecompressTask: public thread_pool::Task {
		DecompressTask() { }
		DecompressTask(gfxlib::Texture &image, const CompressedPart *part)
			:image(&image), part(part) { }
		void Work() {
			char scratch[QLZ_SCRATCH_DECOMPRESS];
			Assert(part->info.x == 0 && part->info.w == image->Width());
			Assert(qlz_size_decompressed(&part->data[0]) == part->info.w * part->info.h * 4);
			//TODO: obsluga dra kawalkow o dowolnej wielkosci i pozycji
		//		for(int y = 0; y < info.h; y++) {
		//			const char *csrc = &decompressed[0] + y * info.w * 4;
		//			char *cdst = (char*)image.DataPointer() + (y + info.y) * image.Pitch() +
		//							info.x * 4;
		//			std::copy(csrc, csrc + info.w * 4, cdst);
		//		}

			char *dst = (char*)image->DataPointer() + part->info.y * image->Pitch();
			int size = qlz_decompress(&part->data[0], dst, scratch);
		}

		gfxlib::Texture *image;
		const CompressedPart *part;
	};

}

int CompressParts(const gfxlib::Texture &image, uint rank, uint nRanks, uint strapHeight,
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
			CompressedPart *part = &parts[num++];
			part->info.x = 0; part->info.w = image.Width(); part->info.y = sy * strapHeight;
			part->info.h = Min(image.Height() - sy * strapHeight, strapHeight);
			part->info.size = part->info.w * part->info.h * 4;
			if(part->data.size() < part->info.size)
				part->data.resize(part->info.size);
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

