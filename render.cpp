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
	RenderTask(const Scene<AccStruct> *sc,const Camera &cam,gfxlib::Texture *tOut,const Options &opt,uint tx,uint ty,
					uint tw,uint th,TreeStats<1> *outSt) :scene(sc),camera(cam),out(tOut),options(opt),
					startX(tx),startY(ty),width(tw),height(th),outStats(outSt) {
		}

	uint startX,startY;
	uint width,height;

	TreeStats<1> *outStats;
	const Scene<AccStruct> *scene;
	Camera camera;
	gfxlib::Texture *out;
	Options options;

	void Work() {
		float ratio = float(out->Width())/float(out->Height());
		enum { NQuads = 1 << (QuadLevels * 2), PWidth = 2 << QuadLevels, PHeight = 2 << QuadLevels };

		Vec3q origin; Broadcast(camera.Pos(), origin);
		Vec3f right, up, front; camera.GetRotation(right, up, front);
		RayGenerator rayGen(QuadLevels, out->Width(), out->Height(), camera.plane_dist, right, up, front);

		uint pitch = out->Pitch();
		u8 *outPtr = ((u8*)out->DataPointer()) + startY * pitch + startX * 4;

		Cache cache;
		Vec3q dir[NQuads], idir[NQuads];

		for(int y = 0; y < height; y += PHeight) {
			for(int x = 0; x < width;x += PWidth) {
				rayGen.Generate(PWidth, PHeight, startX + x, startY + y, dir);
				for(int n = 0; n < NQuads; n++) idir[n] = SafeInv(dir[n]);
			
				Vec3q colors[NQuads];	
				*outStats += scene->RayTrace(RayGroup<1, 0>(&origin, dir, idir, NQuads), cache, colors);

				if(NQuads == 1) {
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

					for(int ty = 0; ty < PHeight; ty++) {
						for(int tx = 0; tx < PWidth; tx += 4) {
							_mm_storeu_si128((__m128i*)dst, ConvColor(*src++).m);
							dst += 16;
						}
						dst += lineDiff;
					}
				}
			}
		}

	}
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

template <int QuadLevels,class AccStruct>
TreeStats<1> Render(const Scene<AccStruct> &scene,const Camera &camera, gfxlib::Texture &image,const Options options,
		uint nThreads) {
	enum { taskSize = 64 };
	
	Assert(image.Width() % 16 == 0);
	Assert(image.GetFormat().BytesPerPixel() == 4);

	uint numTasks = ((image.Width() + taskSize - 1) / taskSize) * ((image.Height() + taskSize - 1) / taskSize);

	vector<TreeStats<1> > taskStats(numTasks);
	vector<RenderTask<AccStruct, QuadLevels>> tasks(numTasks);

	uint num = 0;
	for(uint y = 0; y < image.Height(); y += taskSize)
		for(uint x = 0; x < image.Width(); x += taskSize) {
			tasks[num] = RenderTask<AccStruct,QuadLevels> (&scene, camera, &image, options, x, y,
						Min((int)taskSize, int(image.Width() - x)), Min((int)taskSize, int(image.Height() - y)),
						&taskStats[num]);
			num++;
		}

	Run(tasks, nThreads);

	TreeStats<1> stats;
	for(uint n=0;n<numTasks;n++)
		stats += taskStats[n];

	return stats;
}

template <class AccStruct>
TreeStats<1> Render(const Scene<AccStruct> &scene,const Camera &camera,gfxlib::Texture &image,const Options options,uint tasks) {
	return !gVals[0]?
		Render<4>(scene,camera,image,options,tasks) : Render<3>(scene,camera,image,options,tasks);
}

//typedef bih::Tree<TriangleVector> BIH;

template TreeStats<1> Render<BVH>(const Scene<BVH>&,const Camera&,gfxlib::Texture&,const Options,uint);
//template TreeStats<1> Render<BIH>(const Scene<BIH>&,const Camera&,gfxlib::Texture&,const Options,uint);

//typedef bih::Tree<TreeBoxVector<StaticTree> > FullTree;
//template TreeStats<1> Render<FullTree  >(const Scene<FullTree>  &,const Camera&,gfxlib::Texture&,const Options,uint);
	
