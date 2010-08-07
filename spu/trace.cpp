#include "spu/base.h"
#include "spu/trace.h"

#include "spu/base.cpp"
#include "ray_generator.cpp"

static const float inf = 10E20; // constant::inf

static const Vec3q SafeInv(Vec3q v) {
	return VInv(v + Vec3q(floatq(0.00000001f)));
}

enum {
	QuadLevels = 3,
	NQuads = 1 << (QuadLevels * 2),
	PWidth = 2 << QuadLevels,
	PHeight = 2 << QuadLevels,
};

enum ContextType {
	ctPrimary,
	ctShadow,
	ctSecondary,
};


struct PrimaryContext {
	enum { type = ctPrimary };

	Vec3q rayDir[NQuads], rayIDir[NQuads], rayOrigin;
	Vec2q barycentric[NQuads];
	floatq distance[NQuads];
};

struct ShadowContext {
	enum { type = ctShadow };

	Vec3q rayDir[NQuads], rayIDir[NQuads], rayOrigin;
	floatq distance[NQuads];
};

struct SecondaryContext {
	enum { type = ctSecondary };

	Vec3q rayDir[NQuads], rayIDir[NQuads], rayOrigin[NQuads];
	Vec2q barycentric[NQuads];
	floatq distance[NQuads];
};

Vec3q normals[NQuads];
Vec3q colors[NQuads] ALIGN256;
u8 pixels[NQuads * 4 * 3] ALIGN256;
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

class RayInterval {
public:
	RayInterval(const Vec3q *rayDir, const Vec3q *rayIDir, const Vec3q rayOrigin) {
		ComputeMinMax(rayDir, NQuads, &minDir, &maxDir);
		ComputeMinMax(rayIDir, NQuads, &minIDir, &maxIDir);
		minOrigin = maxOrigin = ExtractN(rayOrigin, 0);
		
		ix = floatq(minIDir.x, maxIDir.x, minIDir.x, maxIDir.x);
		iy = floatq(minIDir.y, maxIDir.y, minIDir.y, maxIDir.y);
		iz = floatq(minIDir.z, maxIDir.z, minIDir.z, maxIDir.z);
	}
	
	floatq ix, iy, iz;
	Vec3f minIDir, maxIDir, minDir, maxDir;
	Vec3f minOrigin, maxOrigin;
};

struct Triangle {
	Vec3f a, ba, ca;
	float t0, it0; int temp[1];
	Vec4f plane;

	template <class Context>
	void Collide(Context &ctx, int idx, int first, int last) const {
		Vec3q tnrm(plane.x, plane.y, plane.z);
		Vec3q ta(a), tca(ca), tba(ba);
		floatq zero(0.0f), one(1.0f);

		Vec3q tvec = ctx.rayOrigin - ta;
		Vec3q tvec0 = (tba ^ tvec) * floatq(it0);
		Vec3q tvec1 = (tvec ^ tca) * floatq(it0);
		floatq tmul = -(tvec | Vec3q(plane.x, plane.y, plane.z));

		for(int q = first; q <= last; q++) {
			const Vec3q dir = ctx.rayDir[q];
			floatq idet = Inv(dir | tnrm);

			floatq dist = idet * tmul;
			floatq v = (dir | tvec0) * idet;
			floatq u = (dir | tvec1) * idet;

			f32x4b test = Min(u, v) >= zero && u + v <= one;
			test = test && idet > zero && dist >= zero && dist < ctx.distance[q];

			ctx.distance[q] = Condition(test, dist, ctx.distance[q]);
			if((ContextType)Context::type != ctShadow)
				normals[q] = Condition(test, tnrm, normals[q]);
		//	c.Object(q) = Condition(i32x4b(test), i32x4(idx), c.Object(q));
		//	c.barycentric[q] = Condition(test, Vec2q(u, v), c.barycentric[q]);
		}
	}

};

struct BBox {
	Vec3f min, max;

	bool TestInterval(const RayInterval &i) const {
		float lmin, lmax;

		float l1, l2, l3, l4;

		l1 = i.minIDir.x * (min.x - i.maxOrigin.x);
		l2 = i.maxIDir.x * (min.x - i.maxOrigin.x);
		l3 = i.minIDir.x * (max.x - i.minOrigin.x);
		l4 = i.maxIDir.x * (max.x - i.minOrigin.x);
		
		lmin = Min(Min(l1, l2), Min(l3, l4));
		lmax = Max(Max(l1, l2), Max(l3, l4));

		l1 = i.minIDir.y * (min.y - i.maxOrigin.y);
		l2 = i.maxIDir.y * (min.y - i.maxOrigin.y);
		l3 = i.minIDir.y * (max.y - i.minOrigin.y);
		l4 = i.maxIDir.y * (max.y - i.minOrigin.y);
		lmin = Max(lmin, Min(Min(l1, l2), Min(l3, l4)));
		lmax = Min(lmax, Max(Max(l1, l2), Max(l3, l4)));

		l1 = i.minIDir.z * (min.z - i.maxOrigin.z);
		l2 = i.maxIDir.z * (min.z - i.maxOrigin.z);
		l3 = i.minIDir.z * (max.z - i.minOrigin.z);
		l4 = i.maxIDir.z * (max.z - i.minOrigin.z);
		lmin = Max(lmin, Min(Min(l1, l2), Min(l3, l4)));
		lmax = Min(lmax, Max(Max(l1, l2), Max(l3, l4)));

		return lmax >= 0.0f && lmin <= lmax;
	}

	template <class Context>
	bool Test(Context &ctx, int &firstActive, int &lastActive) const {
		bool ret = 0;

		Vec3q tmin = Vec3q(min) - ctx.rayOrigin;
		Vec3q tmax = Vec3q(max) - ctx.rayOrigin;

		for(int q = firstActive; q <= lastActive; q++) {
			const Vec3q idir = ctx.rayIDir[q];

			floatq l1 = idir.x * tmin.x;
			floatq l2 = idir.x * tmax.x;
			f32x4b cond = l1 < l2;
			floatq lmin = Condition(cond, l1, l2);
			floatq lmax = Condition(cond, l2, l1);

			l1 = idir.y * tmin.y;
			l2 = idir.y * tmax.y;
			cond = l1 < l2;
			lmin = Max(Condition(cond, l1, l2), lmin);
			lmax = Min(Condition(cond, l2, l1), lmax);

			l1 = idir.z * tmin.z;
			l2 = idir.z * tmax.z;
			cond = l1 < l2;
			lmin = Max(Condition(cond, l1, l2), lmin);
			lmax = Min(Condition(cond, l2, l1), lmax);

			if(ForAny( lmax >= 0.0f && lmin <= Min(lmax, ctx.distance[q]))) {
				firstActive = q;
				ret = 1;
				break;
			}
		}
		for(int q = lastActive; q >= firstActive; q--) {
			const Vec3q idir = ctx.rayIDir[q];

			floatq l1 = idir.x * tmin.x;
			floatq l2 = idir.x * tmax.x;
			f32x4b cond = l1 < l2;
			floatq lmin = Condition(cond, l1, l2);
			floatq lmax = Condition(cond, l2, l1);

			l1 = idir.y * tmin.y;
			l2 = idir.y * tmax.y;
			cond = l1 < l2;
			lmin = Max(Condition(cond, l1, l2), lmin);
			lmax = Min(Condition(cond, l2, l1), lmax);

			l1 = idir.z * tmin.z;
			l2 = idir.z * tmax.z;
			cond = l1 < l2;
			lmin = Max(Condition(cond, l1, l2), lmin);
			lmax = Min(Condition(cond, l2, l1), lmax);

			if(ForAny( lmax >= 0.0f && lmin <= Min(lmax, ctx.distance[q]))) {
				lastActive = q;
				ret = 1;
				break;
			}
		}

		return ret;
	}
};

struct BVHNode {
	BBox bbox;
	union { int subNode, first; };
	union { struct { short firstNode, axis; }; int count; };
		
	bool IsLeaf() const { return subNode & 0x80000000; }
};


template <class T, int tsize>
struct Cache {
	enum {
		size = tsize,
		mask = tsize - 1,
	};
	Cache() :dataPtr(0) { }

	void Init(unsigned long long tdataPtr) {
		dataPtr = tdataPtr;
		for(int n = 0; n < tsize; n++)
			indices[n] = -1;
	}

	void LoadObjects(int first, int count) {
		for(int i = 0; i < count; ) {
			int hash = (first + i) & mask;
			int tcount = size - hash < count - i? size - hash : count - i;
			bool noneed = 1;
			for(int k = 0; k < tcount; k++)
				noneed &= indices[hash + k] == first + i + k;
			if(noneed) {
				i += tcount;
				continue;
			}

			mfc_get(objects + hash, dataPtr + (first + i) * sizeof(T), tcount * sizeof(T), 0, 0, 0);
			mfc_write_tag_mask(1);
			for(int k = 0; k < tcount; k++)
				indices[hash + k] = first + i + k;
			i += tcount;
		}
	}

	void WaitForDMA() {
		mfc_read_tag_status_all();	
	}

	const T& operator[](int idx) {
		int hash = idx & mask;
		if(indices[hash] != idx) {
			Mem2Local(dataPtr + idx * sizeof(T), objects + hash, sizeof(T));
			indices[hash] = idx;
		}
		return objects[hash];
	}

	unsigned long long dataPtr;
	T objects[tsize] ALIGN256;
	int indices[tsize] ALIGN256;
};

Cache<Triangle, 256> triCache;
Cache<BVHNode, 256> bvhCache;

static i32x4 ConvColor(Vec3q rgb) {
	i32x4 tr = Trunc(Clamp(rgb.x * 255.0f, floatq(0.0f), floatq(255.0f)));
	i32x4 tg = Trunc(Clamp(rgb.y * 255.0f, floatq(0.0f), floatq(255.0f)));
	i32x4 tb = Trunc(Clamp(rgb.z * 255.0f, floatq(0.0f), floatq(255.0f)));

	return tb + Shl<8>(tg) + Shl<16>(tr);
}

template <int bpp>
static void StorePixels(const Vec3q *src, u8 *dst) {
	for(int ty = 0; ty < PHeight; ty++) {
		i32x4 ccol;
		for(int tx = 0; tx < PWidth; tx += 4) {
			ccol = ConvColor(*src++);
			for(int k = 0; k < 4; k++) {
				dst[k * 3 + 0] = (ccol[k] & 0xff);
				dst[k * 3 + 1] = (ccol[k] & 0xff00) >> 8;
				dst[k * 3 + 2] = (ccol[k] & 0xff0000) >> 16;
			}
			dst += bpp * 4;
		}
	}
}

struct StackElem {
	StackElem(int node, short first, short last)
		:node(node), first(first), last(last) { }
	StackElem() { }

	int node;
	short first, last;
};

template <class Context>
static void Trace(Context &ctx) {
	StackElem stack[64 + 2]; int stackPos = 0;
	stack[stackPos++] = StackElem(0, 0, NQuads - 1);

	RayInterval interval(ctx.rayDir, ctx.rayIDir, ctx.rayOrigin);

	int sign[3] = { ctx.rayDir[0].x[0] < 0.0f, ctx.rayDir[0].y[0] < 0.0f, ctx.rayDir[0].z[0] < 0.0f };

	while(stackPos) {
		int nNode = stack[--stackPos].node;
		int firstA = stack[stackPos].first;
		int lastA = stack[stackPos].last;

	CONTINUE:
		{
			const BVHNode node = bvhCache[nNode];

			if(node.IsLeaf()) {
				int count = node.count, first = node.first & 0x7fffffff;
				triCache.LoadObjects(first, count);

				if(EXPECT_TAKEN(node.bbox.Test(ctx, firstA, lastA))) {
					triCache.WaitForDMA();
					if(EXPECT_TAKEN(stackPos))
						bvhCache.LoadObjects(stack[stackPos - 1].node, 1);

					for(int n = 0; n < count; n++) {
						const Triangle &tri = triCache[first + n];
						tri.Collide(ctx, first + n, firstA, lastA);
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
}

static void TraceAndShade(PrimaryContext &ctx, Vec3q *__restrict__ colors) {
	for(int n = 0; n < NQuads; n++)
		ctx.distance[n] = floatq(inf);

	Trace(ctx);
	Vec3q diffuse[NQuads], specular[NQuads];
	Vec3q ambient(0.1f, 0.1f, 0.1f);
			
	for(int q = 0; q < NQuads; q++) {
		diffuse[q] = specular[q] = Vec3q(0.0f, 0.0f, 0.0f);
		colors[q] = ctx.rayDir[q] | normals[q];
		colors[q] = Condition(ctx.distance[q] < inf, colors[q], Vec3q(0, 0, 0));
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
		f32x4b anyTest(0, 0, 0, 0);

		for(int q = 0; q < NQuads; q++) {
			Vec3q position = ctx.rayOrigin + ctx.rayDir[q] * ctx.distance[q];
			Vec3q  lightVec = position - lpos;
			f32x4b close = LengthSq(lightVec) < 0.0001f;
			lightVec = Condition(close, Vec3q(0.0f, 1.0f, 0.0f), lightVec);

			distance[q] = Sqrt(lightVec | lightVec);
			shCtx.rayDir[q] = lightVec * Inv(distance[q]);
			shCtx.rayIDir[q] = VInv(shCtx.rayDir[q]);

			dot[q] = normals[q] | shCtx.rayDir[q];
			f32x4b mask = dot[q] > 0.0f && ctx.distance[q] < inf;
			anyTest = anyTest || mask;

			shCtx.distance[q] = Condition(mask, distance[q] * 0.9999f, -inf);
		}

		if(ForAny(anyTest))
			Trace(shCtx);
	
		for(int q = 0; q < NQuads; q++) {
			f32x4  dist = distance[q];
			f32x4b msk  = shCtx.distance[q] == distance[q] * 0.9999f;

			f32x4 atten = dist * iradius;
			atten = Max(f32x4(0.0f), ((floatq(1.0f) - atten) * 0.2f + FastInv(f32x4(16.0f)
							* atten * atten)) - f32x4(0.0625f));

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

	if(info.nLights)
		for(int q = 0; q < NQuads; q++)
			colors[q] = Condition(ctx.distance[q] < inf, colors[q] *
					(diffuse[q] + specular[q] + ambient),
			//	VClamp(diffuse[q] + specular[q] + ambient, Vec3q(0.0f, 0.0f, 0.0f), Vec3q(1.0f, 1.0f, 1.0f)),
				Vec3q(0.0f, 0.0f, 0.0f) );
}

int main(unsigned long long speid, unsigned long long argp, unsigned long long envp) {
	Mem2Local(argp, &info, (sizeof(TaskInfo) + 127) & 0xf80);

	enum { antialias = 1 };
	float scale = antialias? 2 : 1;

	RayGenerator rayGen(QuadLevels, info.outWidth * scale, info.outHeight * scale,
			info.camPlaneDist, info.camRight, info.camUp, info.camFront);

	triCache.Init(info.bvhTris);
	bvhCache.Init(info.bvhNodes);

	for(int y = 0, part = 0; y < info.height; y += PHeight) {
		for(int x = 0; x < info.width; x += PWidth, part++) {
			if(antialias) {
				int offx[4] = { 0, PWidth, 0, PWidth}, offy[4] = {0, 0, PHeight, PHeight};
				int coff[4] = {0, NQuads / 4, NQuads / 4 * 2, NQuads / 4 * 3 };

				Vec3q tcolors[NQuads];

				for(int k = 0; k < 4; k++) {
					PrimaryContext ctx;
					rayGen.Generate(PWidth, PHeight, (info.startX + x) * 2 + offx[k],
							(info.startY + y) * 2 + offy[k], ctx.rayDir);
					ctx.rayOrigin = Vec3q(info.camPos);
					for(int q = 0; q < NQuads; q++)
						ctx.rayIDir[q] = VInv(ctx.rayDir[q]);
					TraceAndShade(ctx, tcolors);
					float *dstx = (float*)&colors[coff[k]].x[0];
					float *dsty = (float*)&colors[coff[k]].y[0];
					float *dstz = (float*)&colors[coff[k]].z[0];

					for(int q = 0; q < NQuads; q += 4) {
						for(int i = 0; i < 4; i++) {
							const Vec3q col = tcolors[q + i];
							dstx[i] = col.x[0] + col.x[1] + col.x[2] + col.x[3];
							dsty[i] = col.y[0] + col.y[1] + col.y[2] + col.y[3];
							dstz[i] = col.z[0] + col.z[1] + col.z[2] + col.z[3];
						}
						dstx += 12; dsty += 12; dstz += 12;
					}
				}
				for(int q = 0; q < NQuads; q++)
					colors[q] *= floatq(0.25f);
			}
			else {
				PrimaryContext ctx;
				rayGen.Generate(PWidth, PHeight, info.startX + x, info.startY + y, ctx.rayDir);
				ctx.rayOrigin = Vec3q(info.camPos);
				for(int q = 0; q < NQuads; q++)
					ctx.rayIDir[q] = VInv(ctx.rayDir[q]);

				TraceAndShade(ctx, colors);
			}

			rayGen.Decompose(colors, colors);
			StorePixels<3>(colors, pixels);
			Local2Mem(info.pixelData + part * NQuads * 3 * 4, pixels, NQuads * 3 * 4);
		}
	}

	return 0;
}
