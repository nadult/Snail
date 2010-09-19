#include <mutex_init.h>
#include <mutex_lock.h>
#include <mutex_unlock.h>

#include <cond_init.h>
#include <cond_signal.h>
#include <cond_wait.h>
#include <cond_broadcast.h>

#include "spu/context.h"
#include "spu/trace.h"
#include <string.h>

#include "pch.h"
#include <iostream>
#include "camera.h"
#include "scene.h"
#include "bvh/tree.h"
#include "dbvh/tree.h"
#include "render.h"

#include "tree_stats.h"
#include "quicklz/quicklz.h"
#include "comm.h"
#include <mpi.h>
#include <time.h>

using comm::MPINode;

static_assert(sizeof(TaskInfo) == 256, "whoops");

//kompresja rozbita na kawalki dzieki czemu mozna kompresowac i przesylac dane
//w trakcie gdy kolejne kawalki sie jeszcze renderuja
#define PROGRESSIVE

namespace {
	
	enum {
	   	maxTasks = 1024 * 16,
	};

	volatile int mutexData __attribute__((aligned(128)));
	volatile int condData1 __attribute__((aligned(128)));
	volatile int condData2 __attribute__((aligned(128)));

	pthread_t threads[maxThreads];
	SPEContext speContext[maxThreads];
	spe_program_handle_t *speProgram = 0;

	volatile TasksInfo tinfo ALIGN256;
	int nThreads = 0;

	volatile TaskInfo tasks[maxTasks] ALIGN256;

	void *Run(void *id) {
		speContext[(long long)id].Run((void*)&tinfo, id);
		return 0;
	}

	void ReloadProgram() {
		if(speProgram)
			spe_image_close(speProgram);

		speProgram = spe_image_open("spu/trace.spu");
		if(!speProgram) {
			printf("Error while loading spu/trace.spu\n");
			speProgram = &spe_trace;
		}
	}

	void InitThreads() {
		static bool init = 0;
		if(init) return;
		init = 1;

		ReloadProgram();

		nThreads = 0;
		tinfo.mutex = (unsigned long long)&mutexData;
		tinfo.finishCond = (unsigned long long)&condData1;
		tinfo.newTaskCond = (unsigned long long)&condData2;

		_mutex_init(tinfo.mutex);
		_cond_init(tinfo.finishCond);
		_cond_init(tinfo.newTaskCond);
		tinfo.nTasks = tinfo.nTaken = tinfo.nAllTasks = 0;
		tinfo.taskList = (unsigned long long)tasks;
	}

	void ChangeThreads(int nth) {
		InputAssert(nth <= maxThreads && nth >= 0);

		while(nThreads < nth) {
			speContext[nThreads].Create();
			speContext[nThreads].Load(speProgram);
			_mutex_lock(tinfo.mutex);
			tinfo.pleaseDie[nThreads] = 0;
			_mutex_unlock(tinfo.mutex);
			pthread_create(&threads[nThreads], 0, Run, (void*)nThreads);
			nThreads++;
		}

		while(nThreads > nth) {
			nThreads--;
			_mutex_lock(tinfo.mutex);
			tinfo.pleaseDie[nThreads] = 1;
			_mutex_unlock(tinfo.mutex);
			pthread_join(threads[nThreads], 0);
			speContext[nThreads].Destroy();
		}
	}

	bool initialized = 0;

};

void KillSPURenderingThreads() {
	ChangeThreads(0);
}

template <class AccStruct>
static TreeStats RenderAndSend(const Scene<AccStruct> &scene,const Camera &camera, uint resx,
		uint resy, unsigned char *data, unsigned char *comprData, const vector<int> &coords,
		const vector<int> &offsets, const Options options, uint rank, uint nThreads) {
	int nTasks = coords.size() / 4;

	if(!initialized) {
		initialized = 1;
		InitThreads();
	}
	if(gVals[3]) {
		static int counter = 0;
		static time_t oldTime;
		if(counter == 0) {
			FileModTime("spu/trace.spu", &oldTime);
			counter++;
		}
		if(counter++ % 64 == 0) {
		//	time_t tTime; FileModTime("spu/trace.spu", &tTime);
		//	if(difftime(tTime, oldTime) > 0) {
		//		oldTime = tTime;
				ChangeThreads(0);
				ReloadProgram();
		//		printf("Reloading spu/trace.spu\n");
		//	}
		}
	}
	ChangeThreads(nThreads);

	vector<TextureInfo, AlignedAllocator<TextureInfo, 256> > texInfo(scene.geometry.materials.size());
	for(int n = 0; n < texInfo.size(); n++) {
		int id = scene.geometry.materials[n].id;
		TextureInfo &info = texInfo[n];
		const gfxlib::Texture *tex = id == ~0? 0 :
			scene.materials[id]?scene.materials[id]->GetTexture() : 0;
		if(tex) {
			Assert(tex->GetFormat().GetBytesPerPixel() == 3);

			info.dataPtr = (unsigned long long)tex->DataPointer();
			info.width = tex->Width();
			info.height = tex->Height();
			Assert(!(info.width & (info.width - 1)));
			Assert(!(info.height & (info.height - 1)));
			info.wMask = info.width - 1;
			info.hMask = info.height - 1;
		}
		else {
			info.dataPtr = info.width = info.height = 0;
		}
	}

	vector<LightData, AlignedAllocator<LightData, 256> > lightData(scene.lights.size());
	for(int n = 0; n < lightData.size(); n++) {
		LightData &lData = lightData[n];
		const Light &light = scene.lights[n];
		lData.pos = light.pos;
		lData.color = light.color;
		lData.radius = light.radius;
		lData.iRadius = light.iRadius;
	}

	_mutex_lock(tinfo.mutex);
	tinfo.nAllTasks = nTasks;
	tinfo.nTasks = 0; tinfo.nTaken = 0;
	tinfo.nTrisIntersected = 0;
	tinfo.nBVHIterations = 0;
	tinfo.nRaysTraced = 0;
	for(int n = 0; n < nThreads; n++)
		tinfo.offsets[n] = 0;
	memset((void*)tinfo.timers, 0, sizeof(tinfo.timers));
	_mutex_unlock(tinfo.mutex);

	for(int n = 0; n < nTasks; n++) {
		int x = coords[n * 4 + 0], y = coords[n * 4 + 1];
		int width = coords[n * 4 + 2], height = coords[n * 4 + 3];

		float ratio = float(resx) / float(resy);
		u8 *outPtr = &data[offsets[n]];
	
		Assert(width == blockWidth && height == blockHeight);

		TaskInfo newTask;
		newTask.bvhNodes = (unsigned long long)&scene.geometry.nodes[0];
		newTask.bvhTris = (unsigned long long)&scene.geometry.tris[0];
		newTask.bvhShTris = (unsigned long long)&scene.geometry.shTris[0];
		newTask.nbvhNodes = scene.geometry.nodes.size();
		newTask.nbvhTris = scene.geometry.tris.size();

		newTask.nLights = lightData.size();
		newTask.lightData = (long long)&lightData[0];
		newTask.texInfoData = (unsigned long long)&texInfo[0];

		newTask.startX = x; newTask.startY = y;
		newTask.width = width; newTask.height = height;
		newTask.outWidth = resx; newTask.outHeight = resy;
		newTask.rData = (unsigned long long)(outPtr + width * height * 0);
		newTask.gData = (unsigned long long)(outPtr + width * height * 1);
		newTask.bData = (unsigned long long)(outPtr + width * height * 2);
		newTask.dataOffset = n;

		newTask.camRight = camera.right;
		newTask.camUp = camera.up;
		newTask.camFront = camera.front;
		newTask.camPos = camera.pos;
		newTask.camPlaneDist = camera.plane_dist;

		newTask.nodeId = rank;
		newTask.colorizeNodes = gVals[8];
		newTask.antialias = gVals[9];
		newTask.reflections = gVals[7];
		newTask.shading = gVals[6];
		newTask.statsVis = gVals[5];
		newTask.compress = !gVals[4];

		_mutex_lock(tinfo.mutex);
		memcpy((void*)(tasks + tinfo.nTasks++), &newTask, sizeof(TaskInfo));
//		_cond_broadcast(tinfo.newTaskCond);
		_mutex_unlock(tinfo.mutex);
	}
	
	TreeStats stats;

	char scratch[QLZ_SCRATCH_COMPRESS];
#ifdef PROGRESSIVE

	int sent = 0;

	bool finished = 0;
	while(!finished) {
		int minOffset = nTasks;
		_mutex_lock(tinfo.mutex);
			for(int k = 0; k < nThreads; k++)
				minOffset = Min(minOffset, tinfo.offsets[k]);
		//	if(minOffset == oldMinOffset) {
		//		_cond_wait(tinfo.finishCond, tinfo.mutex);
		//		for(int k = 0; k < nThreads; k++)
		//			minOffset = Min(minOffset, tinfo.offsets[k]);
		//	}
			finished = tinfo.nFinished == nTasks;
	
			if(finished) {
				tinfo.nFinished = 0;
				tinfo.nTasks = tinfo.nTaken = 0;
				stats.Intersection(tinfo.nTrisIntersected);
				stats.LoopIteration(tinfo.nBVHIterations);
				stats.TracingRays(tinfo.nRaysTraced);
				memcpy(stats.timers, (const void*)tinfo.timers, Min(sizeof(stats.timers), sizeof(tinfo.timers)));
			}
		_mutex_unlock(tinfo.mutex);
		if(finished)
			minOffset = nTasks;

		int packetSize = nTasks / 4;
		while(minOffset > sent && (finished || minOffset - sent >= packetSize)) {
			char buf[16 + packetSize * (16 + blockWidth * blockHeight * 3)];

			int nParts = Min(minOffset - sent, (int)packetSize);
			((int*)buf)[0] = ByteSwap(nParts);
			((int*)buf)[1] = ((int*)buf)[2] = ((int*)buf)[3] = 0;
			memcpy(buf + 4, data + 4 + 16 * sent, nParts * 16); 
			char *tdata = buf + 4 + nParts * 16;

			memcpy(buf + 16 + nParts * 16, data + offsets[sent], nParts * blockWidth * blockHeight * 3);
			int csize = qlz_compress(buf, (char*)comprData, 16 + nParts * (16 + blockWidth * blockHeight * 3),
										scratch);
			int tcsize = ByteSwap(csize);
		//	MPINode(0, 0) << tcsize << comm::Data(comprData, csize);
			// TODO: Buforowanie przeszkadza na scenkach z teksturkami
			MPI_Bsend(&tcsize, sizeof(int), MPI_CHAR, 0, 0, MPI_COMM_WORLD);
			MPI_Bsend(comprData, csize, MPI_CHAR, 0, 1, MPI_COMM_WORLD);
			sent += nParts;
		}
	}

#else

	_mutex_lock(tinfo.mutex);
		if(tinfo.nFinished != nTasks)
			_cond_wait(tinfo.finishCond, tinfo.mutex);
		tinfo.nFinished = 0;
		tinfo.nTasks = tinfo.nTaken = 0;
		stats.Intersection(tinfo.nTrisIntersected);
		stats.LoopIteration(tinfo.nBVHIterations);
		stats.TracingRays(tinfo.nRaysTraced);
		memcpy(stats.timers, (const void*)tinfo.timers, Min(sizeof(stats.timers), sizeof(tinfo.timers)));
	_mutex_unlock(tinfo.mutex);
	
	if(gVals[4]) {
		int csize = offsets[nTasks - 1] + blockWidth * blockHeight * 3;
		int tcsize = ByteSwap(-csize);
		MPINode(0, 0) << tcsize << comm::Data(data, csize);
	}
	else {
		int csize = qlz_compress((char*)data, (char*)comprData,
							offsets[nTasks - 1] + blockWidth * blockHeight * 3, scratch);
		int tcsize = ByteSwap(csize);
		MPINode(0, 0) << tcsize << comm::Data(comprData, csize);
	}
#endif

	return stats;
}

template TreeStats RenderAndSend<BVH>(const Scene<BVH>&, const Camera&, uint, uint, unsigned char*,
			unsigned char*, const vector<int>&, const vector<int>&, const Options, uint, uint);

