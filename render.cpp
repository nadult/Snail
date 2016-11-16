#include <iostream>
#include "camera.h"
#include "scene.h"
#include "bvh/tree.h"
#include "dbvh/tree.h"
#include "render.h"
#include "thread_pool.h"
#include "ray_generator.h"
#include "fwk_gfx.h"
#include "mipmap_texture.h"

static i32x4 ConvColor(Vec3q rgb) {
	i32x4 tr = Trunc(Clamp(rgb.x * 255.0f, floatq(0.0f), floatq(255.0f)));
	i32x4 tg = Trunc(Clamp(rgb.y * 255.0f, floatq(0.0f), floatq(255.0f)));
	i32x4 tb = Trunc(Clamp(rgb.z * 255.0f, floatq(0.0f), floatq(255.0f)));

	return tb + Shl<8>(tg) + Shl<16>(tr);
}

template <class AccStruct,int QuadLevels>
struct RenderTask: public thread_pool::Task {
	RenderTask() { }
	RenderTask(const Scene<AccStruct> *sc, const Camera &cam, uint resx, uint resy, uint pitch, unsigned char *dst,
		const Options &opt, uint tx, uint ty, uint tw, uint th, uint rank, TreeStats *outSt, bool compress)
		:scene(sc), camera(cam), resx(resx), resy(resy), pitch(pitch), out(dst), options(opt),
		startX(tx), startY(ty), width(tw), height(th), rank(rank), outStats(outSt), compress(compress)
	{ }

	uint resx, resy, pitch;
	uint startX, startY;
	uint width, height;
	uint rank;

	TreeStats *outStats;
	const Scene<AccStruct> *scene;
	unsigned char *out;
	Camera camera;
	Options options;
	bool compress;

	enum {
		colorizeNodes = 1
	};

#if defined(__PPC) || defined(__PPC__)
	void Work(SPEContext *context) {
#else
	void Work() {
#endif
		float ratio = float(resx) / float(resy);
		enum {
			NQuads = 1 << (QuadLevels * 2),
			PWidth = 2 << QuadLevels,
			PHeight = 2 << QuadLevels,
	   		maxWidth = 16,
			maxHeight = 64,
		};
		u8 *outPtr = out;
		Vec3q origin; Broadcast(camera.pos, origin);
	
		float scale = gVals[9]? 2 : 1;
		RayGenerator rayGen(QuadLevels, resx * scale, resy * scale,
							camera.plane_dist, camera.right, camera.up, camera.front);

		Cache cache;
		Vec3q dir[NQuads], idir[NQuads];

		for(int y = 0; y < height; y += PHeight) {
			for(int x = 0; x < width; x += PWidth) {
				Vec3q colors[NQuads];	

				if(gVals[9]) {
					int offx[4] = { 0, PWidth, 0, PWidth }, offy[4] = { 0, 0, PHeight, PHeight };
					int coff[4] = { 0, 2, NQuads / 2, NQuads / 2 + 2 };

					for(int k = 0; k < 4; k++) {
						rayGen.Generate(PWidth, PHeight, (startX + x) * 2 + offx[k],
								(startY + y) * 2 + offy[k], dir);
						for(int n = 0; n < NQuads; n++) idir[n] = SafeInv(dir[n]);
					
						Vec3q tcolors[NQuads];
						*outStats += scene->RayTrace(RayGroup<1, 0>(&origin, dir, idir, NQuads), cache, tcolors);
						enum { line = 4 };
							
						float *dstx = (float*)&colors[coff[k]].x[0];
						float *dsty = (float*)&colors[coff[k]].y[0];
						float *dstz = (float*)&colors[coff[k]].z[0];

						for(int tq = 0; tq < NQuads; tq += line * 2) {

							for(int t = 0; t < line; t += 2) {
								int q = tq + t;

								for(int i = 0; i < 2; i++) {
									const Vec3q col = (tcolors[q + i] + tcolors[q + i + line]) * floatq(0.25f);

									dstx[i * 2 + 0] = col.x[0] + col.x[1];
									dsty[i * 2 + 0] = col.y[0] + col.y[1];
									dstz[i * 2 + 0] = col.z[0] + col.z[1];
									dstx[i * 2 + 1] = col.x[2] + col.x[3];
									dsty[i * 2 + 1] = col.y[2] + col.y[3];
									dstz[i * 2 + 1] = col.z[2] + col.z[3];
								}
								dstx += 12; dsty += 12; dstz += 12;
							}
							dstx += 12 * (line / 2);
							dsty += 12 * (line / 2);
							dstz += 12 * (line / 2);
						}
					}
				}
				else {
					rayGen.Generate(PWidth, PHeight, startX + x, startY + y, dir);
					for(int n = 0; n < NQuads; n++) idir[n] = SafeInv(dir[n]);
			
					*outStats += scene->RayTrace(RayGroup<1, 0>(&origin, dir, idir, NQuads), cache, colors);
				}

				if(colorizeNodes && gVals[8]) {
					Vec3f ncolors[] = {
						Vec3f(0.6, 0.6, 0.6), Vec3f(0.6, 0.6, 1.0),
						Vec3f(0.6, 1.0, 0.6), Vec3f(0.6, 1.0, 1.0),
						Vec3f(1.0, 0.6, 0.6), Vec3f(1.0, 0.6, 1.0),
						Vec3f(1.0, 1.0, 0.6), Vec3f(1.0, 1.0, 1.0),
						Vec3f(0.3, 0.3, 0.3), Vec3f(0.3, 0.3, 1.0),
						Vec3f(0.3, 0.7, 0.3), Vec3f(0.3, 0.7, 0.7),
						Vec3f(0.7, 0.3, 0.3), Vec3f(0.7, 0.3, 0.7),
						Vec3f(0.7, 0.7, 0.3), Vec3f(0.7, 1.7, 0.7),
					};
					Vec3q color(ncolors[rank % (sizeof(ncolors) / sizeof(Vec3f))]);
					for(int q = 0; q < NQuads; q++)
						colors[q] = (colors[q] + Vec3q(0.1f, 0.1f, 0.1f)) * color;
				}

				if(NQuads == 1)
					throw 0; //TODO
				else {
			//		rayGen.Decompose(colors, colors);
					const Vec3q *src = colors;

					if(compress) {
						if(width != maxWidth || height != maxHeight) {
							//TODO
							return;
						}

						for(int ty = 0; ty < PHeight; ty++) {
							unsigned char *dstr = outPtr + (y + ty) * width + x + width * height * 0;
							unsigned char *dstg = outPtr + (y + ty) * width + x + width * height * 1;
							unsigned char *dstb = outPtr + (y + ty) * width + x + width * height * 2;

							for(int tx = 0; tx < PWidth; tx += 4) {
								i32x4 tr = Trunc(Clamp(src->x * 255.0f, floatq(0.0f), floatq(255.0f)));
								i32x4 tg = Trunc(Clamp(src->y * 255.0f, floatq(0.0f), floatq(255.0f)));
								i32x4 tb = Trunc(Clamp(src->z * 255.0f, floatq(0.0f), floatq(255.0f)));
								src++;

								dstr[0] = tr[0]; dstr[1] = tr[1]; dstr[2] = tr[2]; dstr[3] = tr[3];
								dstg[0] = tg[0]; dstg[1] = tg[1]; dstg[2] = tg[2]; dstg[3] = tg[3];
								dstb[0] = tb[0]; dstb[1] = tb[1]; dstb[2] = tb[2]; dstb[3] = tb[3];
								dstg[0] -= dstr[0]; dstb[0] -= dstr[0];
								dstg[1] -= dstr[1]; dstb[1] -= dstr[1];
								dstg[2] -= dstr[2]; dstb[2] -= dstr[2];
								dstg[3] -= dstr[3]; dstb[3] -= dstr[3];

								dstr += 4;
								dstg += 4;
								dstb += 4;
							}
						}
					}
					else {
						int lineDiff = pitch - PWidth * 3;
						unsigned char *dst = outPtr + x * 3 + y * pitch;
						int pheight = Min(height - y, (uint)PHeight);
						int pwidth = Min(width - x, (uint)PWidth);

						for(int ty = 0; ty < pheight; ty++) {
							i32x4 ccol;
							for(int tx = 0; tx + 3 < pwidth; tx += 4) {
								ccol = ConvColor(*src++);
								*(int*)(dst + 3 * 0) = ccol[0];
								*(int*)(dst + 3 * 1) = ccol[1];
								*(int*)(dst + 3 * 2) = ccol[2];
								dst[3 * 3 + 0] = ccol[3] & 0xff;
								dst[3 * 3 + 1] = (ccol[3] >> 8 ) & 0xff;
								dst[3 * 3 + 2] = (ccol[3] >> 16) & 0xff;
								dst += 3 * 4;
							}
							if(pwidth & 3) {
								ccol = ConvColor(*src++);
								*(int*)(dst + 3 * 0) = ccol[0];
								if((pwidth & 3) > 1) *(int*)(dst + 3 * 1) = ccol[0];
								if((pwidth & 3) > 2) *(int*)(dst + 3 * 2) = ccol[0];
								dst += 3 * 4;
							}
							dst += lineDiff;
						}
					}

					/*
					if(compress)
						StorePixels<1>(colors, outPtr, x, y, Min(width - x, (uint)PWidth),
										Min(height - y, (uint)PHeight), pitch);
					else
						StorePixels<0>(colors, outPtr, x, y, Min(width - x, (uint)PWidth),
										Min(height - y, (uint)PHeight), pitch); */
				}
			}
		}

	}
};

template <int QuadLevels, class AccStruct>
static TreeStats Render(const Scene<AccStruct> &scene, const Camera &camera,
		MipmapTexture &image, const Options options, uint nThreads) {
	enum { taskSize = 64 };
	ASSERT(image.GetFormat() == fwk::TextureFormatId::rgb);

	uint nTasks = ((image.Width() + taskSize - 1) / taskSize)
					* ((image.Height() + taskSize - 1) / taskSize);

	vector<TreeStats> taskStats(nTasks);
	vector<RenderTask<AccStruct, QuadLevels>> tasks(nTasks);

	uint num = 0;
	for(uint y = 0; y < image.Height(); y += taskSize)
		for(uint x = 0; x < image.Width(); x += taskSize) {
			tasks[num] = RenderTask<AccStruct,QuadLevels> (&scene, camera, image.Width(), image.Height(),
						image.Pitch(), (unsigned char*)image.DataPointer() + x * 3 + image.Pitch() * y,
						options, x, y, Min((int)taskSize, int(image.Width() - x)),
						Min((int)taskSize, int(image.Height() - y)), 0, &taskStats[num], 0);
			num++;
	}
	Run(tasks, nThreads);

	TreeStats stats;
	for(uint n = 0; n < nTasks; n++)
		stats += taskStats[n];

	return stats;
}

template <int QuadLevels,class AccStruct>
static TreeStats Render(const Scene<AccStruct> &scene,const Camera &camera, uint resx, uint resy,
			unsigned char *data, const vector<int> &coords, const vector<int> &offsets,
			const Options options, uint rank, uint nThreads) {
	int nTasks = coords.size() / 4;

	vector<TreeStats> taskStats(nTasks);
	vector<RenderTask<AccStruct, QuadLevels>> tasks(nTasks);

	for(int n = 0; n < nTasks; n++) {
		int x = coords[n * 4 + 0], y = coords[n * 4 + 1];
		int w = coords[n * 4 + 2], h = coords[n * 4 + 3];
		tasks[n] = RenderTask<AccStruct,QuadLevels> (&scene, camera, resx, resy, w * 3, &data[offsets[n]],
													options, x, y, w, h, rank, &taskStats[n], 1);
	}

	thread_pool::Run(tasks, nThreads);

	TreeStats stats;
	for(int n = 0; n < nTasks; n++)
		stats += taskStats[n];

	return stats;
}

template <class AccStruct>
TreeStats Render(const Scene<AccStruct> &scene, const Camera &camera, uint resx, uint resy,
				unsigned char *data, const vector<int> &coords, const vector<int> &offsets,
				const Options options, uint rank, uint threads) {
	return Render<3>(scene, camera, resx, resy, data, coords, offsets, options, rank, threads);
}

template <class AccStruct>
TreeStats Render(const Scene<AccStruct> &scene, const Camera &camera,
				MipmapTexture &image, const Options options, uint threads) {
	return Render<3>(scene, camera, image, options, threads);
}


template TreeStats Render<BVH>(const Scene<BVH>&, const Camera&, uint, uint, unsigned char*,
								const vector<int>&, const vector<int>&, const Options, uint, uint);

template TreeStats Render<BVH>(const Scene<BVH>&, const Camera&, MipmapTexture&,
									const Options, uint);

template TreeStats Render<DBVH>(const Scene<DBVH>&, const Camera&, uint, uint, unsigned char*,
								const vector<int>&, const vector<int>&, const Options, uint, uint);

template TreeStats Render<DBVH>(const Scene<DBVH>&, const Camera&, MipmapTexture&,
									const Options, uint);
	

