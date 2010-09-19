#ifndef RTRACER_SPU_TRIANGLE_H
#define RTRACER_SPU_TRIANGLE_H

#include "spu/base.h"

class RayInterval;

struct Triangle {
	Vec3f a, ba, ca;
	float t0, it0; int temp[1];
	Vec4f plane;

	void Collide(PrimaryContext &ctx, int idx, int first, int last) const;
	void Collide(ShadowContext &ctx, int idx, int first, int last) const;
	void Collide(SecondaryContext &ctx, int idx, int first, int last) const;

	template <class Context>
	inline void Prepare(const Context &ctx, Vec3q &tnrm, Vec3q &tvec0, Vec3q &tvec1, floatq &tmul) const;
	inline void Prepare(const SecondaryContext &ctx, Vec3q &tnrm, Vec3q &ta, Vec3q &tba, Vec3q &tca, floatq &tit0) const;

	bool TestInterval(const RayInterval &i) const;
};

void MultiCollide(const Triangle &a, const Triangle &b, const Triangle &c, const Triangle &d,
					PrimaryContext &ctx, int idx, int first, int last);
void MultiCollide(const Triangle &a, const Triangle &b, const Triangle &c, const Triangle &d,
					ShadowContext &ctx, int idx, int first, int last);
void MultiCollide(const Triangle &a, const Triangle &b, const Triangle &c, const Triangle &d,
					SecondaryContext &ctx, int idx, int first, int last);

template <class Context>
void MultiCollide(const Triangle &a, const Triangle &b, const Triangle &c, const Triangle &d,
					Context &ctx, int idx, int first, int last) {
	a.Collide(ctx, idx + 0, first, last);
	b.Collide(ctx, idx + 1, first, last);
	c.Collide(ctx, idx + 2, first, last);
	d.Collide(ctx, idx + 3, first, last);
}

struct ShTriangle {
	Vec2f uv[3];
	Vec3f nrm[3];
	int matId;
};

static_assert(sizeof(Triangle) == 64, "");
static_assert(sizeof(ShTriangle) == 64, "");


#endif

