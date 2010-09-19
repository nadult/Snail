#include "spu/base.h"
#include "spu/trace.h"
#include "spu/triangle.h"
#include "spu/bbox.h"
#include "spu/texture.h"
#include "spu/stats.h"

#include "ray_generator.cpp"
#include <cassert>


#define COMPRESS

void Compress(unsigned char*);

static const float inf = 10E20; // constant::inf

static const Vec3q SafeInv(Vec3q v) {
	return VInv(v + Vec3q(floatq(0.00000001f)));
}

TaskInfo info ALIGN256;

void ComputeMinMax(const Vec3q *vec, int size, Vec3f *outMin, Vec3f *outMax) {
	Vec3q min = vec[0], max = vec[0];
	for(int q = 1; q < size; q++) {
		min = VMin(min, vec[q]);
		max = VMax(max, vec[q]);
	}
	
	*outMin = Minimize(min);
	*outMax = Maximize(max);
}

RayInterval::RayInterval(const Vec3q *rayDir, const Vec3q *rayIDir, const Vec3q rayOrigin, int count) {
	ComputeMinMax(rayDir, count, &minDir, &maxDir);
	ComputeMinMax(rayIDir, count, &minIDir, &maxIDir);
	minOrigin = maxOrigin = ExtractN(rayOrigin, 0);
	
	ix = floatq(minIDir.x, maxIDir.x, minIDir.x, maxIDir.x);
	iy = floatq(minIDir.y, maxIDir.y, minIDir.y, maxIDir.y);
	iz = floatq(minIDir.z, maxIDir.z, minIDir.z, maxIDir.z);
}

RayInterval::RayInterval(const Vec3q *rayDir, const Vec3q *rayIDir, const Vec3q* rayOrigin, int count) {
	ComputeMinMax(rayDir, count, &minDir, &maxDir);
	ComputeMinMax(rayIDir, count, &minIDir, &maxIDir);
	ComputeMinMax(rayOrigin, count, &minOrigin, &maxOrigin);
	
	ix = floatq(minIDir.x, maxIDir.x, minIDir.x, maxIDir.x);
	iy = floatq(minIDir.y, maxIDir.y, minIDir.y, maxIDir.y);
	iz = floatq(minIDir.z, maxIDir.z, minIDir.z, maxIDir.z);
}

struct BVHNode {
	BBox bbox;
	union { int subNode, first; };
	short firstNode, axis;
//	union { struct { short firstNode, axis; }; int count; };
		
	bool IsLeaf() const { return subNode & 0x80000000; }
};

Cache<Triangle, 256, 1> triCache;
Cache<ShTriangle, 256, 2> shTriCache;
Cache<BVHNode, 512, 3> bvhCache;
Stats stats;
int gTimers[16] = {0, };

struct StackElem {
	StackElem(int node, short first, short last)
		:node(node), first(first), last(last) { }
	StackElem() { }

	int node;
	short first, last;
};

template <class Context>
static void Trace(Context &ctx) {
	stats.rays += NQuads * 4;
	BlockTimer timer(timerTracing);

	StackElem stack[64 + 2]; int stackPos = 0;
	stack[stackPos++] = StackElem(0, 0, NQuads - 1);

	RayInterval interval(ctx.rayDir, ctx.rayIDir, ctx.rayOrigin, NQuads);
	int sign[3] = { ctx.rayDir[0].x[0] < 0.0f, ctx.rayDir[0].y[0] < 0.0f, ctx.rayDir[0].z[0] < 0.0f };

	while(stackPos) {
		int nNode = stack[--stackPos].node;
		int firstA = stack[stackPos].first;
		int lastA = stack[stackPos].last;
		stats.iters++;

	CONTINUE:
		{
			const BVHNode node = bvhCache[nNode];

			if(node.IsLeaf()) {
				union { short ts[2]; int count; };
				ts[0] = node.firstNode; ts[1] = node.axis;
				int first = node.first & 0x7fffffff;

				triCache.LoadObjects(first, count);
				bool test = node.bbox.Test(ctx, firstA, lastA);
				triCache.WaitForDMA();

				if(test) {
					if(EXPECT_TAKEN(stackPos))
						bvhCache.LoadObjects(stack[stackPos - 1].node, 1);

					{
						stats.intersects += count;
						BlockTimer timer(timerIntersecting);

						int n = 0, tcount = count & ~3;
						for(; n < tcount; n += 4)
							MultiCollide(triCache(first + n), triCache(first + n + 1), triCache(first + n + 2),
										 triCache(first + n + 3), ctx, first + n, firstA, lastA);
						for(; n < count; n++) 
							triCache(first + n).Collide(ctx, first + n, firstA, lastA);
					}
					bvhCache.WaitForDMA();
				}
				continue;
			}

			int child = node.subNode;
			if(EXPECT_TAKEN(!node.bbox.TestInterval(interval)))
				continue;
			bvhCache.LoadObjects(child, 2);

			if(EXPECT_TAKEN(node.bbox.Test(ctx, firstA, lastA))) {
				int firstNode = node.firstNode ^ sign[node.axis];
				stack[stackPos++] = StackElem(child + (firstNode ^ 1), firstA, lastA);
				nNode = child + firstNode;

				bvhCache.WaitForDMA();
				goto CONTINUE;
			}
			bvhCache.WaitForDMA();
		}
	}
	
	if((ContextType)Context::type == ctShadow) {
		BlockTimer shTimer(5);
		shTimer.timestamp = timer.timestamp;
	}
}

const Vec3q Reflect(const Vec3q ray, const Vec3q nrm) {
	floatq dot = (nrm | ray);
	return ray - nrm * (dot + dot);
}

inline const Vec3q RayOrigin(const PrimaryContext &ctx, int q) { return ctx.rayOrigin; }
inline const Vec3q RayOrigin(const SecondaryContext &ctx, int q) { return ctx.rayOrigin[q]; }

inline const Vec2q MulAdd(const Vec2q v1, floatq mul, const Vec2q add) {
	return Vec2q(
			MulAdd(v1.x, mul, add.x),
			MulAdd(v1.y, mul, add.y));
}
	
inline Vec3q MulAdd(const Vec3q v1, floatq mul, const Vec3q add) {
	return Vec3q(
			MulAdd(v1.x, mul, add.x),
			MulAdd(v1.y, mul, add.y),
			MulAdd(v1.z, mul, add.z) );
}

template <class Context>
static void TraceAndShade(Context &ctx) {
	Stats oldStats = stats;
	BlockTimer shTimer(timerShading);
	int oldTimerTracing = gTimers[timerTracing];

	Trace(ctx);
	Vec3q diffuse[NQuads], specular[NQuads];
	Vec3q ambient(0.1f, 0.1f, 0.1f);
	
	if(info.shading) for(int q = 0; q < NQuads; q++) {
		diffuse[q] = specular[q] = Vec3q(0.0f, 0.0f, 0.0f);
		Vec3q normal = ctx.normals[q];

		i32x4 matIds(~0);
		i32x4 triIds = Condition(i32x4b(ctx.distance[q] < inf), ctx.triIds[q], i32x4(~0));
		int tri0 = triIds[0];
		Vec2q uv(0.0f, 0.0f);

		if(ForAll(triIds == i32x4(tri0))) {
			if(EXPECT_TAKEN( tri0 != ~0 )) {
				const ShTriangle &shTri = shTriCache[tri0];

				uv = Vec2q(shTri.uv[0]) +
					MulAdd(Vec2q(shTri.uv[1]), ctx.barycentric[q].x,
							Vec2q(shTri.uv[2]) * ctx.barycentric[q].y);
				normal = Vec3q(shTri.nrm[0]) +
						MulAdd(Vec3q(shTri.nrm[1]), ctx.barycentric[q].x,
							Vec3q(shTri.nrm[2]) * ctx.barycentric[q].y);

				matIds = shTri.matId & 0x7fffffff;
			}
		}
		else {
			for(int k = 0; k < 4; k++) if(EXPECT_TAKEN(triIds[k] != ~0)) {
				const ShTriangle &shTri = shTriCache[triIds[k]];
				Vec2f bar(ctx.barycentric[q].x[k], ctx.barycentric[q].y[k]);
				uv.x[k] = shTri.uv[0].x + bar.x * shTri.uv[1].x + bar.y * shTri.uv[2].x;
				uv.y[k] = shTri.uv[0].y + bar.x * shTri.uv[1].y + bar.y * shTri.uv[2].y;
				
				normal.x[k] =	shTri.nrm[0].x + shTri.nrm[1].x * ctx.barycentric[q].x[k] +
								shTri.nrm[2].x * ctx.barycentric[q].y[k];
				normal.y[k] =	shTri.nrm[0].y + shTri.nrm[1].y * ctx.barycentric[q].x[k] +
								shTri.nrm[2].y * ctx.barycentric[q].y[k];
				normal.z[k] =	shTri.nrm[0].z + shTri.nrm[1].z * ctx.barycentric[q].x[k] +
								shTri.nrm[2].z * ctx.barycentric[q].y[k];
				matIds[k] = shTri.matId & 0x7fffffff;
			}
		}

		Vec3q color = ctx.rayDir[q] | normal;
		ctx.normals[q] = normal;

		int mat0 = matIds[0];
		bool singleMat = ForAll(matIds == i32x4(mat0));

		if(EXPECT_TAKEN( singleMat )) {
			color *= SampleTexture(mat0, uv);
		}
		else {
			for(int k = 0; k < 4; k++) {
				Vec3q tcolor = SampleTexture(matIds[k], uv);
				color.x[k] *= tcolor.x[0];
				color.y[k] *= tcolor.y[0];
				color.z[k] *= tcolor.z[0];
			}
		}
		
		ctx.colors[q] = Condition(ctx.distance[q] < inf, color);
	}
	else for(int q = 0; q < NQuads; q++) {
		diffuse[q] = specular[q] = Vec3q(0.0f, 0.0f, 0.0f);
		ctx.colors[q] = Condition(ctx.distance[q] < inf,
			(Vec3q)(ctx.rayDir[q] | ctx.normals[q]), Vec3q(0, 0, 0));
	}

	for(int n = 0; n < info.nLights; n++) {
		LightData lightData;
		Mem2Local(info.lightData + n * sizeof(LightData), &lightData, sizeof(LightData));

		Vec3q lpos(lightData.pos), lcol(lightData.color);
		floatq radius(lightData.radius), iradius(lightData.iRadius);
		floatq dot[NQuads], distance[NQuads];
		Vec3q fromLight[NQuads];

		ShadowContext shCtx;
		shCtx.rayOrigin = lpos;
		f32x4b anyTest(false);
		f32x4 zero(0.0f), one(1.0f), oneEps(0.9999f);

		for(int q = 0; q < NQuads; q++) {
			Vec3q position = RayOrigin(ctx, q) + ctx.rayDir[q] * ctx.distance[q];
			Vec3q  lightVec = position - lpos;
			f32x4b close = LengthSq(lightVec) < 0.0001f;
			lightVec = Condition(close, Vec3q(zero, one, zero), lightVec);

			distance[q] = Sqrt(lightVec | lightVec);
			shCtx.rayDir[q] = lightVec * Inv(distance[q]);
			shCtx.rayIDir[q] = VInv(shCtx.rayDir[q]);

			dot[q] = ctx.normals[q] | shCtx.rayDir[q];
			f32x4b mask = dot[q] > zero && ctx.distance[q] < inf; // TODO: a co jak liczymy odbicia i dist == -inf?
			anyTest = anyTest || mask;

			shCtx.distance[q] = Condition(mask, distance[q] * oneEps, -inf);
		}

		if(ForAny(anyTest)) {
			Trace(shCtx);

			for(int q = 0; q < NQuads; q++) {
				f32x4  dist = distance[q];
				f32x4b msk  = shCtx.distance[q] == distance[q] * oneEps;

				f32x4 atten = dist * iradius;
				atten = Max(zero, ((one - atten) * 0.2f + FastInv(f32x4(16.0f) * atten * atten)) - f32x4(0.0625f));

				f32x4 diffMul = dot[q] * atten;
				f32x4 specMul = dot[q];
				specMul *= specMul;
				specMul *= specMul;
				specMul *= specMul;
				specMul *= specMul;
				specMul *= atten;

				diffuse[q] += Condition(msk, lcol * diffMul);
				specular[q] += Condition(msk, lcol * specMul);
			}
		}
	}

	if(info.reflections && ctx.reflections < 1) {
		SecondaryContext sctx;
		sctx.reflections = ctx.reflections + 1;
		f32x4b any(false);

		for(int q = 0; q < NQuads; q++) {
			f32x4b cond = ctx.distance[q] < inf;
			any = any || cond;
			sctx.distance[q] = Condition(cond, inf, -inf);
			sctx.rayDir[q] = Reflect(ctx.rayDir[q], ctx.normals[q]);
			sctx.rayIDir[q] = SafeInv(sctx.rayDir[q]);
			sctx.rayOrigin[q] = RayOrigin(ctx, q) + ctx.rayDir[q] * ctx.distance[q]
								+ sctx.rayDir[q] * floatq(0.0001f);
		}
		if(ForAny(any)) {
			TraceAndShade(sctx);
			for(int q = 0; q < NQuads; q++)
				ctx.colors[q] = Condition(ctx.distance[q] < inf,
						ctx.colors[q] * floatq(0.7f) + sctx.colors[q] * floatq(0.3f), ctx.colors[q]);
		}
	}

	if(info.nLights) {
		for(int q = 0; q < NQuads; q++)
			ctx.colors[q] = Condition(ctx.distance[q] < inf, ctx.colors[q] *
					(diffuse[q] + specular[q] + ambient),
			//	VClamp(diffuse[q] + specular[q] + ambient, Vec3q(0.0f, 0.0f, 0.0f), Vec3q(1.0f, 1.0f, 1.0f)),
				Vec3q(0.0f, 0.0f, 0.0f) );
	}

	if(info.statsVis) {
		Stats dstats = stats - oldStats;
		Vec3q color( 0.0f,
				float(dstats.iters) * 0.002,
				float(dstats.intersects) * 0.002);

		for(int q = 0; q < NQuads; q++)
			ctx.colors[q] = color;
	}

	gTimers[timerShading] -= gTimers[timerTracing] - oldTimerTracing;
}

u8 red[blockWidth * blockHeight] ALIGN256;
u8 green[blockWidth * blockHeight] ALIGN256;
u8 blue[blockWidth * blockHeight] ALIGN256;

template <bool compress>
static void StorePixels(int sx, int sy, Vec3q *src) {
	unsigned char *sdstr = red + sx + sy * blockWidth;
	unsigned char *sdstg = green + sx + sy * blockWidth;
	unsigned char *sdstb = blue + sx + sy * blockWidth;

	for(int ty = 0; ty < PHeight; ty++) {
		unsigned char *dstr = sdstr + ty * blockWidth;
		unsigned char *dstg = sdstg + ty * blockWidth;
		unsigned char *dstb = sdstb + ty * blockWidth;

		for(int tx = 0; tx < PWidth; tx += 4) {
			i32x4 tr = Trunc(Clamp(src->x * 255.0f, floatq(0.0f), floatq(255.0f)));
			i32x4 tg = Trunc(Clamp(src->y * 255.0f, floatq(0.0f), floatq(255.0f)));
			i32x4 tb = Trunc(Clamp(src->z * 255.0f, floatq(0.0f), floatq(255.0f)));
			src++;

			dstr[0] = tr[0]; dstr[1] = tr[1]; dstr[2] = tr[2]; dstr[3] = tr[3];
			dstg[0] = tg[0]; dstg[1] = tg[1]; dstg[2] = tg[2]; dstg[3] = tg[3];
			dstb[0] = tb[0]; dstb[1] = tb[1]; dstb[2] = tb[2]; dstb[3] = tb[3];

			if(compress) {
				dstg[0] -= dstr[0]; dstb[0] -= dstr[0];
				dstg[1] -= dstr[1]; dstb[1] -= dstr[1];
				dstg[2] -= dstr[2]; dstb[2] -= dstr[2];
				dstg[3] -= dstr[3]; dstb[3] -= dstr[3];
			}

			dstr += 4;
			dstg += 4;
			dstb += 4;
		}
	}
}

int ProcessTask() {
	BlockTimer timer(timerRendering);

	float scale = info.antialias? 2 : 1;

	RayGenerator rayGen(QuadLevels, (int)(info.outWidth * scale), (int)(info.outHeight * scale),
			info.camPlaneDist, info.camRight, info.camUp, info.camFront);

	//TODO: reinicjowac tylko jesli sie zmienily dane; choc duzo to raczej nie da
	triCache.Init(info.bvhTris);
	bvhCache.Init(info.bvhNodes);
	shTriCache.Init(info.bvhShTris);
	InitTexCache(info.texInfoData);

	for(int y = 0, part = 0; y < info.height; y += PHeight) {
		for(int x = 0; x < info.width; x += PWidth, part++) {
			PrimaryContext ctx;

			if(info.antialias) {
				int offx[4] = { 0, PWidth, 0, PWidth }, offy[4] = { 0, 0, PHeight, PHeight };
				int coff[4] = { 0, NQuads / 4, NQuads / 4 * 2, NQuads / 4 * 3 };

				for(int k = 0; k < 4; k++) {
					PrimaryContext mctx;
					rayGen.Generate(PWidth, PHeight, (info.startX + x) * 2 + offx[k],
							(info.startY + y) * 2 + offy[k], mctx.rayDir);
					mctx.rayOrigin = Vec3q(info.camPos);
					for(int q = 0; q < NQuads; q++)
						mctx.rayIDir[q] = VInv(mctx.rayDir[q]);
					for(int n = 0; n < NQuads; n++)
						mctx.distance[n] = floatq(inf);
					TraceAndShade(mctx);
					float *dstx = (float*)&ctx.colors[coff[k]].x[0];
					float *dsty = (float*)&ctx.colors[coff[k]].y[0];
					float *dstz = (float*)&ctx.colors[coff[k]].z[0];

					for(int q = 0; q < NQuads; q += 4) {
						for(int i = 0; i < 4; i++) {
							const Vec3q col = mctx.colors[q + i];
							dstx[i] = col.x[0] + col.x[1] + col.x[2] + col.x[3];
							dsty[i] = col.y[0] + col.y[1] + col.y[2] + col.y[3];
							dstz[i] = col.z[0] + col.z[1] + col.z[2] + col.z[3];
						}
						dstx += 12; dsty += 12; dstz += 12;
					}
				}
				for(int q = 0; q < NQuads; q++)
					ctx.colors[q] *= floatq(0.25f);
			}
			else {
				rayGen.Generate(PWidth, PHeight, info.startX + x, info.startY + y, ctx.rayDir);
				ctx.rayOrigin = Vec3q(info.camPos);
				for(int q = 0; q < NQuads; q++)
					ctx.rayIDir[q] = VInv(ctx.rayDir[q]);
				for(int n = 0; n < NQuads; n++)
					ctx.distance[n] = floatq(inf);

				TraceAndShade(ctx);
			}

			if(info.colorizeNodes) {
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
				Vec3q color(ncolors[info.nodeId % (sizeof(ncolors) / sizeof(Vec3f))]);

				for(int q = 0; q < NQuads; q++)
					ctx.colors[q] = (ctx.colors[q] + Vec3q(0.1f, 0.1f, 0.1f)) * color;
			}

			rayGen.Decompose(ctx.colors, ctx.colors);
			if(info.compress) StorePixels<1>(x, y, ctx.colors);
			else StorePixels<0>(x, y, ctx.colors);
		}
	}

	assert(info.width == blockWidth && info.height == blockHeight);
	assert((info.rData & 0xf) ||(info.gData & 0xf) ||(info.bData & 0xf));

	Compress((unsigned char*)red);
	Local2Mem(info.rData, red, blockWidth * blockHeight);

	Compress((unsigned char*)green);
	Local2Mem(info.gData, green, blockWidth * blockHeight);

	Compress((unsigned char*)blue);
	Local2Mem(info.bData, blue, blockWidth * blockHeight);

	return 0;
}

