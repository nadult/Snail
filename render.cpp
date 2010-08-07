#if defined(__PPC) || defined(__PPC__)
	#define SPURENDER
#endif

#include "pch.h"
#include <iostream>
#include "camera.h"
#include "scene.h"
#include "bvh/tree.h"
#include "dbvh/tree.h"
#include "render.h"
#include "thread_pool.h"

#ifdef SPURENDER

#include "spu/context.h"
#include "spu/trace.h"
#include <string.h>

#else

#include "ray_generator.h"

#endif

static i32x4 ConvColor(Vec3q rgb) {
	i32x4 tr = Trunc(Clamp(rgb.x * 255.0f, floatq(0.0f), floatq(255.0f)));
	i32x4 tg = Trunc(Clamp(rgb.y * 255.0f, floatq(0.0f), floatq(255.0f)));
	i32x4 tb = Trunc(Clamp(rgb.z * 255.0f, floatq(0.0f), floatq(255.0f)));

	return tb + Shl<8>(tg) + Shl<16>(tr);
}

template <int bpp>
static void StorePixels(const Vec3q *src, u8 *dst, int width, int height, int pitch) {
	static_assert(bpp == 4 || bpp == 3, "if(width & 3) { ... } needs to be modified for bpp < 3");
	int lineDiff = pitch - width * bpp;

	for(int ty = 0; ty < height; ty++) {
		i32x4 ccol;
		for(int tx = 0; tx + 3 < width; tx += 4) {
			if(bpp == 4) {
#ifdef VECLIB_SSE
				_mm_storeu_si128((__m128i*)dst, ConvColor(*src++).m);
#else
				ThrowException("StorePixels<4> needs implementation for ppc");
#endif
			}
			else {
				ccol = ConvColor(*src++);
#ifdef __BIG_ENDIAN
				ccol[0] = ByteSwap(ccol[0]);
				ccol[1] = ByteSwap(ccol[1]);
				ccol[2] = ByteSwap(ccol[2]);
//				ccol[3] = ByteSwap(ccol[3]);
#endif
				*(int*)(dst + bpp * 0) = ccol[0];
				*(int*)(dst + bpp * 1) = ccol[1];
				*(int*)(dst + bpp * 2) = ccol[2];
				dst[bpp * 3 + 0] = ccol[3] & 0xff;
				dst[bpp * 3 + 1] = (ccol[3] >> 8 ) & 0xff;
				dst[bpp * 3 + 2] = (ccol[3] >> 16) & 0xff;
			}
			dst += bpp * 4;
		}
		if(width & 3) {
			ccol = ConvColor(*src++);
#ifdef __BIG_ENDIAN
//			ccol[0] = ByteSwap(ccol[0]);
#endif
			*(int*)(dst + bpp * 0) = ccol[0];
			if((width & 3) > 1) *(int*)(dst + bpp * 1) = ccol[0];
			if((width & 3) > 2) *(int*)(dst + bpp * 2) = ccol[0];
			dst += bpp * 4;
		}
		dst += lineDiff;
	}
}


template <class AccStruct,int QuadLevels>
struct RenderTask: public thread_pool::Task {
	RenderTask() { }
	RenderTask(const Scene<AccStruct> *sc, const Camera &cam, gfxlib::Texture *tOut,
			const Options &opt, uint tx, uint ty, uint tw, uint th, uint rank, TreeStats *outSt)
		:scene(sc), camera(cam), out(tOut), options(opt), startX(tx), startY(ty),
		width(tw), height(th), rank(rank), outStats(outSt)
	{
#ifdef SPURENDER
		preload = &spe_trace;
#endif
	}

	uint startX, startY;
	uint width, height;
	uint rank;

	TreeStats *outStats;
	const Scene<AccStruct> *scene;
	gfxlib::Texture *out;
	Camera camera;
	Options options;

	enum {
		colorizeNodes = 1
	};


#if defined(__PPC) || defined(__PPC__)
	void Work(SPEContext *context) {
#else
	void Work() {
#endif
		float ratio = float(out->Width()) / float(out->Height());
		enum { NQuads = 1 << (QuadLevels * 2), PWidth = 2 << QuadLevels, PHeight = 2 << QuadLevels };
		uint pitch = out->Pitch(), bpp = out->GetFormat().BytesPerPixel();
		u8 *outPtr = ((u8*)out->DataPointer()) + startY * pitch + startX * bpp;
	
#ifdef SPURENDER
		InputAssert(bpp == 3);

		int nParts = ((width + PWidth - 1) / PWidth) * ((height + PHeight - 1) / PHeight);
		TaskInfo tinfo ALIGN256;
		u8 pixelData[NQuads * 4 * 3 * nParts] ALIGN256;

		vector<LightData, AlignedAllocator<LightData, 256> > lightData(scene->lights.size());
		for(int n = 0; n < lightData.size(); n++) {
			LightData &lData = lightData[n];
			const Light &light = scene->lights[n];
			lData.pos = light.pos;
			lData.color = light.color;
			lData.radius = light.radius;
			lData.iRadius = light.iRadius;
		}

		tinfo.bvhNodes = (unsigned long long)&scene->geometry.nodes[0];
		tinfo.bvhTris = (unsigned long long)&scene->geometry.triCache[0];
		tinfo.nbvhNodes = scene->geometry.nodes.size();
		tinfo.nbvhTris = scene->geometry.elements.size();

		tinfo.nLights = lightData.size();
		tinfo.lightData = (long long)&lightData[0];

		tinfo.startX = startX; tinfo.startY = startY;
		tinfo.width = width; tinfo.height = height;
		tinfo.outWidth = out->Width();
		tinfo.outHeight = out->Height();
		tinfo.nParts = nParts; tinfo.pixelData = (long long)pixelData;
		camera.GetRotation(tinfo.camRight, tinfo.camUp, tinfo.camFront);
		tinfo.camPos = camera.Pos();
		tinfo.camPlaneDist = camera.plane_dist;

		context->Run(&tinfo, 0);

		for(int y = 0, part = 0; y < height; y += PHeight) {
			for(int x = 0; x < width; x += PWidth, part++) {
				const u8 *src = pixelData + part * NQuads * 4 * 3;
				u8 *dst = outPtr + x * 3 + y * pitch;

				uint ey = Min(height - y, (uint)PHeight);
				uint line = Min(width  - x, (uint)PWidth) * 3;
				for(int ty = 0; ty < ey; ty++)
					memcpy(dst + ty * pitch, src + ty * PWidth * 3, line);
			}
		}

#else
		Vec3q origin; Broadcast(camera.Pos(), origin);
		Vec3f right, up, front; camera.GetRotation(right, up, front);
		RayGenerator rayGen(QuadLevels, out->Width(), out->Height(), camera.plane_dist,
							right, up, front);


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
						Vec3f(1.0, 0.6, 0.6),
						Vec3f(0.6, 1.0, 0.6),
						Vec3f(0.6, 0.6, 1.0),
						Vec3f(1.0, 1.0, 0.6),
						Vec3f(1.0, 0.6, 1.0),
						Vec3f(0.6, 1.0, 1.0),
						Vec3f(1.0, 1.0, 1.0) };
					Vec3q color(ncolors[rank % (sizeof(ncolors) / sizeof(Vec3f))]);
					for(int q = 0; q < NQuads; q++)
						colors[q] = (colors[q] + Vec3q(0.1f, 0.1f, 0.1f)) * color;
				}

				if(NQuads == 1)
					throw 0; //TODO
				else {
					rayGen.Decompose(colors, colors);
					if(bpp == 3) StorePixels<3>(colors, outPtr + x * bpp + y * pitch, Min(width - x, (uint)PWidth),
									Min(height - y, (uint)PHeight), pitch);
					else  StorePixels<4>(colors, outPtr + x * bpp + y * pitch, Min(width - x, (uint)PWidth),
									Min(height - y, (uint)PHeight), pitch);
				}
			}
		}
#endif

	}
};

template <int QuadLevels, class AccStruct>
TreeStats Render(const Scene<AccStruct> &scene, const Camera &camera,
		gfxlib::Texture &image, const Options options, uint nThreads) {
	enum { taskSize = 64 };

	uint nTasks = ((image.Width() + taskSize - 1) / taskSize) * ((image.Height() + taskSize - 1) / taskSize);

	vector<TreeStats> taskStats(nTasks);
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

	TreeStats stats;
	for(uint n = 0; n < nTasks; n++)
		stats += taskStats[n];

	return stats;
}

template <int QuadLevels,class AccStruct>
TreeStats Render(const Scene<AccStruct> &scene,const Camera &camera,
		gfxlib::Texture &image, uint rank, uint nRanks, uint strapHeight,
		const Options options, uint nThreads) {
	enum { taskSize = 128 };

	vector<TreeStats> taskStats;
	vector<RenderTask<AccStruct, QuadLevels>> tasks;
	tasks.reserve(1024);
	taskStats.reserve(1024);

	uint num = 0;

	for(uint sy = 0; sy < (image.Height() + strapHeight - 1) / strapHeight; sy++)
		if(sy % nRanks == rank) {
			for(uint x = 0; x < image.Width(); x += taskSize) {
				uint y = sy * strapHeight;

				taskStats.push_back(TreeStats());
				tasks.push_back(
					RenderTask<AccStruct,QuadLevels> (&scene, camera, &image, options,
						x, y, Min((int)taskSize, int(image.Width() - x)),
						Min((int)strapHeight, int(image.Height() - y)), rank, &taskStats[num]) );
				num++;
			}
		}

	thread_pool::Run(tasks, nThreads);

	TreeStats stats;
	for(uint n=0;n < tasks.size();n++)
		stats += taskStats[n];

	return stats;
}

template <class AccStruct>
TreeStats Render(const Scene<AccStruct> &scene, const Camera &camera,
				gfxlib::Texture &image, uint rank, uint nRanks, uint strapHeight,
				const Options options, uint threads) {
	return Render<3>(scene, camera, image, rank, nRanks, strapHeight, options, threads);
}

template <class AccStruct>
TreeStats Render(const Scene<AccStruct> &scene, const Camera &camera,
				gfxlib::Texture &image, const Options options, uint threads) {
	return Render<3>(scene, camera, image, options, threads);
}


template TreeStats Render<BVH>(const Scene<BVH>&, const Camera&, gfxlib::Texture&,
									uint, uint, uint, const Options, uint);

template TreeStats Render<BVH>(const Scene<BVH>&, const Camera&, gfxlib::Texture&,
									const Options, uint);
	

//template TreeStats Render<DBVH>(const Scene<DBVH>&, const Camera&, gfxlib::Texture&,
//									uint, uint, uint, const Options, uint);

//template TreeStats Render<DBVH>(const Scene<DBVH>&, const Camera&, gfxlib::Texture&,
//									const Options, uint);
	

