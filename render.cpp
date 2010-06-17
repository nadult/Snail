#include <iostream>
#include "ray_generator.h"
#include "scene_inl.h"
#include "camera.h"

#include "bih/tree.h"
#include "gl_window.h"
#include "formats/loader.h"
#include "base_scene.h"
#include "tree_box.h"
#include "font.h"
#include "render.h"

#include <pthread.h>

inline static i32x4 ConvColor(Vec3q rgb) {
	i32x4 tr=Trunc(Clamp(rgb.x*255.0f,floatq(0.0f),floatq(255.0f)));
	i32x4 tg=Trunc(Clamp(rgb.y*255.0f,floatq(0.0f),floatq(255.0f)));
	i32x4 tb=Trunc(Clamp(rgb.z*255.0f,floatq(0.0f),floatq(255.0f)));

	__m128i c1,c2,c3,c4;
	c1=_mm_packs_epi32(tr.m,tr.m);
	c2=_mm_packs_epi32(tg.m,tg.m);
	c3=_mm_packs_epi32(tb.m,tb.m);
	c1=_mm_packus_epi16(c1,c1);
	c2=_mm_packus_epi16(c2,c2);
	c3=_mm_packus_epi16(c3,c3);

	__m128i c12=_mm_unpacklo_epi8(c1,c2);
	__m128i c33=_mm_unpacklo_epi8(c3,c3);
	return _mm_unpacklo_epi16(c12,c33);
}

struct Task {
	virtual ~Task() { }
	virtual void Work() = 0;
};

template <class AccStruct,int QuadLevels>
struct RenderTask: public Task {
	RenderTask() { }
	RenderTask(const Scene<AccStruct> *sc,const Camera &cam,Image *tOut,const Options &opt,uint tx,uint ty,
					uint tw,uint th,TreeStats<1> *outSt) :scene(sc),camera(cam),out(tOut),options(opt),
					startX(tx),startY(ty),width(tw),height(th),outStats(outSt) {
		}

	uint startX,startY;
	uint width,height;

	TreeStats<1> *outStats;
	const Scene<AccStruct> *scene;
	Camera camera;
	Image *out;
	Options options;

	void Work() {
		float ratio=float(out->width)/float(out->height);

		enum { NQuads=1<<(QuadLevels*2), PWidth=2<<QuadLevels, PHeight=2<<QuadLevels };

		Vec3q origin; Broadcast(camera.pos, origin);
		RayGenerator rayGen(QuadLevels, out->width, out->height, camera.plane_dist);

		uint pitch=out->width*3;
		u8 *outPtr=(u8*)&out->buffer[startY * pitch + startX * 3];

		Cache cache;
		Vec3q dir[NQuads], idir[NQuads];

		for(int y=0;y<height;y+=PHeight) {
			for(int x=0;x<width;x+=PWidth) {
				rayGen.Generate(PWidth, PHeight, startX + x, startY + y, dir);

				{
					Vec3q ax(camera.right), ay(camera.up), az(camera.front);
					for(int n = 0; n < NQuads; n++) {
						dir[n] = ax * dir[n].x + ay * dir[n].y + az * dir[n].z;
						idir[n] = SafeInv(dir[n]);
					}
				}
				
				Result<NQuads> result =
					scene->RayTrace(RayGroup<NQuads,1>(&origin,dir,idir),FullSelector<NQuads>(),cache);
				*outStats += result.stats;

				if(NQuads==1) {
					union { __m128i icol; u8 c[16]; };
					icol = ConvColor(result.color[0]).m;

					u8 *p1=outPtr+y*pitch+x*3;
					u8 *p2=p1+pitch;

					p1[ 0]=c[ 0]; p1[ 1]=c[ 1]; p1[ 2]=c[ 2];
					p1[ 3]=c[ 4]; p1[ 4]=c[ 5]; p1[ 5]=c[ 6];
					p2[ 0]=c[ 8]; p2[ 1]=c[ 9]; p2[ 2]=c[10];
					p2[ 3]=c[12]; p2[ 4]=c[13]; p2[ 5]=c[14];
				}
				else {
					rayGen.Decompose(result.color, result.color);

					Vec3q *src = result.color;
					u8 *dst=outPtr+x*3+y*pitch;
					int lineDiff=pitch-PWidth*3;

					for(int ty=0;ty<PHeight;ty++) {
						for(int tx=0;tx<PWidth;tx+=4) {
							union { __m128i icol; u8 c[16]; };
							icol = ConvColor(*src++).m;
	
							dst[ 0]=c[ 0]; dst[ 1]=c[ 1]; dst[ 2]=c[ 2];
							dst[ 3]=c[ 4]; dst[ 4]=c[ 5]; dst[ 5]=c[ 6];
							dst[ 6]=c[ 8]; dst[ 7]=c[ 9]; dst[ 8]=c[10];
							dst[ 9]=c[12]; dst[10]=c[13]; dst[11]=c[14];

							dst+=12;
						}
						dst+=lineDiff;
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
TreeStats<1> Render(const Scene<AccStruct> &scene,const Camera &camera,Image &image,const Options options,
		uint nThreads) {
	enum { taskSize = 64 };

	uint numTasks = ((image.width + taskSize - 1) / taskSize) * ((image.height + taskSize - 1) / taskSize);

	vector<TreeStats<1> > taskStats(numTasks);
	vector<RenderTask<AccStruct, QuadLevels>> tasks(numTasks);

	uint num = 0;
	for(uint y = 0;y < image.height; y += taskSize)
		for(uint x = 0;x < image.width; x += taskSize) {
			tasks[num] = RenderTask<AccStruct,QuadLevels> (&scene,camera,&image,options,x,y,
						Min((int)taskSize,int(image.width - x)),Min((int)taskSize,int(image.height - y)),
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
TreeStats<1> Render(const Scene<AccStruct> &scene,const Camera &camera,Image &image,const Options options,uint tasks) {
	return Render<3>(scene,camera,image,options,tasks);
}

typedef bih::Tree<TriangleVector> StaticTree;
typedef bih::Tree<TreeBoxVector<StaticTree> > FullTree;

template TreeStats<1> Render<StaticTree>(const Scene<StaticTree>&,const Camera&,Image&,const Options,uint);
//template TreeStats<1> Render<FullTree  >(const Scene<FullTree>  &,const Camera&,Image&,const Options,uint);
	
