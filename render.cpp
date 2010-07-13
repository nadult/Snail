#include "pch.h"
#include <iostream>
#include "ray_generator.h"
#include "scene_inl.h"
#include "camera.h"

#include "bvh/tree.h"
//#include "bih/tree.h"
#include "gl_window.h"
#include "formats/loader.h"
#include "base_scene.h"
#include "font.h"
#include "render.h"

#include <pthread.h>
#include "quicklz/quicklz.h"

static i32x4 ConvColor(Vec3q rgb) {
	i32x4 tr = Trunc(Clamp(rgb.x * 255.0f, floatq(0.0f), floatq(255.0f)));
	i32x4 tg = Trunc(Clamp(rgb.y * 255.0f, floatq(0.0f), floatq(255.0f)));
	i32x4 tb = Trunc(Clamp(rgb.z * 255.0f, floatq(0.0f), floatq(255.0f)));

	return tr + Shl<8>(tg) + Shl<16>(tb);
}

struct Task {
	virtual ~Task() { }
	virtual void Work() = 0;
};

template <class AccStruct,int QuadLevels>
struct RenderTask: public Task {
	RenderTask() { }
	RenderTask(const Scene<AccStruct> *sc, const Camera &cam, gfxlib::Texture *tOut,
			const Options &opt, uint tx, uint ty, uint tw, uint th, uint rank, TreeStats<1> *outSt)
		:scene(sc), camera(cam), out(tOut), options(opt), startX(tx), startY(ty),
		width(tw), height(th), rank(rank), outStats(outSt) { }

	uint startX, startY;
	uint width, height;
	uint rank;

	TreeStats<1> *outStats;
	const Scene<AccStruct> *scene;
	gfxlib::Texture *out;
	Camera camera;
	Options options;

	enum {
		colorizeNodes = 1
	};

	void Work() {
		float ratio = float(out->Width()) / float(out->Height());
		enum { NQuads = 1 << (QuadLevels * 2), PWidth = 2 << QuadLevels, PHeight = 2 << QuadLevels };

		Vec3q origin; Broadcast(camera.Pos(), origin);
		Vec3f right, up, front; camera.GetRotation(right, up, front);
		RayGenerator rayGen(QuadLevels, out->Width(), out->Height(), camera.plane_dist,
							right, up, front);

		uint pitch = out->Pitch();
		u8 *outPtr = ((u8*)out->DataPointer()) + startY * pitch + startX * 4;

		Cache cache;
		Vec3q dir[NQuads], idir[NQuads];

		for(int y = 0; y < height; y += PHeight) {
			for(int x = 0; x < width; x += PWidth) {
				rayGen.Generate(PWidth, PHeight, startX + x, startY + y, dir);
				for(int n = 0; n < NQuads; n++) idir[n] = SafeInv(dir[n]);
			
				Vec3q colors[NQuads];	
				*outStats += scene->RayTrace(RayGroup<1, 0>(&origin, dir, idir, NQuads), cache, colors);

				if(colorizeNodes && gVals[8]) {
					Vec3f ncolors[] = {
						Vec3f(1, 0, 0),
						Vec3f(0, 1, 0),
						Vec3f(0, 0, 1),
						Vec3f(1, 1, 0),
						Vec3f(1, 0, 1),
						Vec3f(0, 1, 1),
						Vec3f(1, 1, 1) };
					Vec3q color(ncolors[rank % (sizeof(ncolors) / sizeof(Vec3f))]);
					for(int q = 0; q < NQuads; q++)
						colors[q] *= color;
				}

				if(NQuads == 1) {
					exit(0); //TODO
					union { __m128i icol; u8 c[16]; };
					icol = ConvColor(colors[0]).m;

					u8 *p1 = outPtr + y * pitch + x * 4;
					u8 *p2 = p1 + pitch;

					p1[ 0] = c[ 0]; p1[ 1] = c[ 1]; p1[ 2] = c[ 2];
					p1[ 4] = c[ 4]; p1[ 5] = c[ 5]; p1[ 6] = c[ 6];
					p2[ 0] = c[ 8]; p2[ 1] = c[ 9]; p2[ 2] = c[10];
					p2[ 4] = c[12]; p2[ 5] = c[13]; p2[ 6] = c[14];
				}
				else {
					rayGen.Decompose(colors, colors);

					const Vec3q *src = colors;
					u8 *dst = outPtr + x * 4 + y * pitch;
					int lineDiff = pitch - PWidth * 4;

					int ex = Min(width - x, PWidth), ey = Min(height - y, PHeight);
					for(int ty = 0; ty < ey; ty++) {
						for(int tx = 0; tx + 3 < ex; tx += 4) {
							_mm_storeu_si128((__m128i*)dst, ConvColor(*src++).m);
							dst += 16;
						}
						if(ex & 3) {
							union { __m128i mi; int i4[4]; };
							mi = ConvColor(*src++).m;
							int *idst = (int*)dst;
							idst[0] = i4[0];
							if((ex & 3) > 1) idst[1] = i4[1];
							if((ex & 3) > 2) idst[2] = i4[2];
							dst += 16;
						}
						dst += lineDiff;
					}
				}
			}
		}

	}
};

struct CompressTask: public Task {
	CompressTask(const gfxlib::Texture &image, CompressedPart *part) :image(&image), part(part) { }
	void Work() {
		char scratch[QLZ_SCRATCH_COMPRESS];
		const char *src = (const char*)image->DataPointer() + part->info.y * image->Pitch();
		part->info.size = qlz_compress(src, &part->data[0], part->info.size, scratch);
	}

	const gfxlib::Texture *image;
	CompressedPart *part;
};

#include <iostream>

enum { maxThreads = 32 };

static pthread_t threads[maxThreads];
static pthread_mutex_t mutexes[maxThreads];
static pthread_cond_t sleeping[maxThreads];
static pthread_spinlock_t spinlock;
static volatile bool diePlease[maxThreads];
static volatile int nextTask = 0, finished = 0;
static vector<Task*> tasks;
static int nThreads = 0;

static void *InnerLoop(void *id_) {	
	int id = (long long)id_;
REPEAT:
	while(true) {
		Task *task;

		pthread_spin_lock(&spinlock);
		if(nextTask == tasks.size()) {
			finished |= (1 << id);
			pthread_spin_unlock(&spinlock);
			break;
		}
		task = tasks[nextTask++];
		pthread_spin_unlock(&spinlock);

		task->Work();
	}

	if(id && !diePlease[id - 1]) {
		pthread_mutex_lock(&mutexes[id - 1]);
		pthread_cond_wait(&sleeping[id - 1], &mutexes[id - 1]);
		pthread_mutex_unlock(&mutexes[id - 1]);
		goto REPEAT;
	}

	return 0;
}

static void FreeThreads();

static void ChangeThreads(int nThreads_) {
	Assert(nThreads_ > 0 && nThreads_ < maxThreads);

	static bool sinit = 0;
	if(!sinit) {
		pthread_spin_init(&spinlock, PTHREAD_PROCESS_PRIVATE);
		atexit(FreeThreads);
		sinit = 1;
	}
	nThreads_--;

	for(; nThreads < nThreads_; nThreads++) {
		diePlease[nThreads] = 0;
		pthread_mutex_init(&mutexes[nThreads], 0);
		pthread_cond_init(&sleeping[nThreads], 0);
		pthread_create(&threads[nThreads], 0, InnerLoop, (void*)(nThreads + 1));
	}
	while(nThreads > nThreads_) {
		nThreads--;
		diePlease[nThreads] = 1;
		pthread_cond_signal(&sleeping[nThreads]);
		pthread_join(threads[nThreads], 0);
		pthread_mutex_destroy(&mutexes[nThreads]);
		pthread_cond_destroy(&sleeping[nThreads]);
	}
}

void FreeThreads() {
	ChangeThreads(1);
	pthread_spin_destroy(&spinlock);
}

template <class TTask>
static void Run(vector<TTask> &ttasks, int nThreads_) {
	ChangeThreads(nThreads_);

	pthread_spin_lock(&spinlock);
		tasks.resize(ttasks.size());
		for(int n = 0; n < ttasks.size(); n++)
			tasks[n] = (Task*)&(ttasks[n]);
		finished = 0;
		nextTask = 0;
	pthread_spin_unlock(&spinlock);

	for(int n = 0; n < nThreads; n++)
		pthread_cond_broadcast(&sleeping[n]);
	InnerLoop(0);

	bool end = 0;
	while(!end) {
		pthread_spin_lock(&spinlock);
		if(finished == (1 << ((long long)(nThreads + 1))) - 1) {
			end = 1;
			tasks.clear();
			nextTask = 0;
		}
		pthread_spin_unlock(&spinlock);
		usleep(100);
	}
}

template <int QuadLevels, class AccStruct>
TreeStats<1> Render(const Scene<AccStruct> &scene, const Camera &camera,
		gfxlib::Texture &image, const Options options, uint nThreads) {
	enum { taskSize = 64 };
	Assert(image.GetFormat().BytesPerPixel() == 4);

	uint nTasks = ((image.Width() + taskSize - 1) / taskSize) * ((image.Height() + taskSize - 1) / taskSize);

	vector<TreeStats<1> > taskStats(nTasks);
	vector<RenderTask<AccStruct, QuadLevels>> tasks(nTasks);

	uint num = 0;
	for(uint y = 0; y < image.Height(); y += taskSize)
		for(uint x = 0; x < image.Width(); x += taskSize) {
			tasks[num] = RenderTask<AccStruct,QuadLevels> (&scene, camera, &image, options, x, y,
						Min((int)taskSize, int(image.Width() - x)), Min((int)taskSize, int(image.Height() - y)),
						0, &taskStats[num]);
			num++;
	}
	Run(tasks, nThreads);

	TreeStats<1> stats;
	for(uint n = 0; n < nTasks; n++)
		stats += taskStats[n];

	return stats;
}


template <int QuadLevels,class AccStruct>
TreeStats<1> Render(const Scene<AccStruct> &scene,const Camera &camera,
		gfxlib::Texture &image, uint rank, uint nRanks, uint strapHeight,
		const Options options, uint nThreads) {
	enum { taskSize = 64 };
	Assert(image.GetFormat().BytesPerPixel() == 4);

	vector<TreeStats<1> > taskStats;
	vector<RenderTask<AccStruct, QuadLevels>> tasks;
	tasks.reserve(1024);
	taskStats.reserve(1024);

	uint num = 0;

	for(uint sy = 0; sy < (image.Height() + strapHeight - 1) / strapHeight; sy++)
		if(sy % nRanks == rank) {
			for(uint x = 0; x < image.Width(); x += taskSize) {
				uint y = sy * strapHeight;

				taskStats.push_back(TreeStats<1>());
				tasks.push_back(
					RenderTask<AccStruct,QuadLevels> (&scene, camera, &image, options,
						x, y, Min((int)taskSize, int(image.Width() - x)),
						Min((int)strapHeight, int(image.Height() - y)), rank, &taskStats[num]) );
				num++;
			}
		}

	Run(tasks, nThreads);

	TreeStats<1> stats;
	for(uint n=0;n < tasks.size();n++)
		stats += taskStats[n];

	return stats;
}

template <class AccStruct>
TreeStats<1> Render(const Scene<AccStruct> &scene, const Camera &camera,
				gfxlib::Texture &image, uint rank, uint nRanks, uint strapHeight,
				const Options options, uint threads) {
	return !gVals[0]?
		Render<2>(scene, camera, image, rank, nRanks, strapHeight, options, threads) :
		Render<3>(scene, camera, image, rank, nRanks, strapHeight, options, threads);
}

template <class AccStruct>
TreeStats<1> Render(const Scene<AccStruct> &scene, const Camera &camera,
				gfxlib::Texture &image, const Options options, uint threads) {
	return !gVals[0]?
		Render<2>(scene, camera, image, options, threads) :
		Render<3>(scene, camera, image, options, threads);
}

template TreeStats<1> Render<BVH>(const Scene<BVH>&, const Camera&, gfxlib::Texture&,
									uint, uint, uint, const Options, uint);
template TreeStats<1> Render<BVH>(const Scene<BVH>&, const Camera&, gfxlib::Texture&,
									const Options, uint);
	

int CompressParts(const gfxlib::Texture &image, uint rank, uint nRanks, uint strapHeight,
				uint nThreads, vector<CompressedPart> &buffers) {
	vector<CompressTask> tasks;
	tasks.reserve(128);
	int num = 0;
	for(uint sy = 0; sy < (image.Height() + strapHeight - 1) / strapHeight; sy++)
		if(sy % nRanks == rank)
			num++;
	if(buffers.size() < num)
		buffers.resize(num);
	num = 0;

	for(uint sy = 0; sy < (image.Height() + strapHeight - 1) / strapHeight; sy++)
		if(sy % nRanks == rank) {
			int height = Min(image.Height() - sy * strapHeight, strapHeight);
			CompressedPart *part = &buffers[num++];
			part->info.x = 0; part->info.w = image.Width(); part->info.y = sy * strapHeight;
			part->info.h = Min(image.Height() - sy * strapHeight, strapHeight);
			part->info.size = part->info.w * part->info.h * 4;
			if(part->data.size() < part->info.size)
				part->data.resize(part->info.size);
			tasks.push_back(CompressTask(image, part));
		}

	Run(tasks, nThreads);
	return num;
}
