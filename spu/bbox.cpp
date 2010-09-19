#include "spu/bbox.h"
#include "spu/trace.h"
#include "spu/stats.h"

bool BBox::TestInterval(const RayInterval &i) const { // 4%
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
bool BBox::Test(Context &ctx, int &firstActive, int &lastActive) const { // 25%
	bool ret = 0;

	Vec3q tmin = Vec3q(min) - ctx.rayOrigin;
	Vec3q tmax = Vec3q(max) - ctx.rayOrigin;
	floatq zero(0.0f);

#define COMPUTE(tidir, lmin, lmax) { const Vec3q idir = tidir; \
		floatq l1 = idir.x * tmin.x; \
		floatq l2 = idir.x * tmax.x; \
		floatq l3 = idir.y * tmin.y; \
		floatq l4 = idir.y * tmax.y; \
		floatq l5 = idir.z * tmin.z; \
		floatq l6 = idir.z * tmax.z; \
		f32x4b cond1 = l1 < l2; \
		f32x4b cond2 = l3 < l4; \
		f32x4b cond3 = l5 < l6; \
		floatq lmin1 = Condition(cond1, l1, l2); \
		floatq lmax1 = Condition(cond1, l2, l1); \
		floatq lmin2 = Condition(cond2, l3, l4); \
		floatq lmax2 = Condition(cond2, l4, l3); \
		lmin = Max(lmin1, lmin2); \
		lmax = Min(lmax1, lmax2); \
		floatq lmin3 = Condition(cond3, l5, l6); \
		floatq lmax3 = Condition(cond3, l6, l5); \
		lmin = Max(lmin, lmin3); \
		lmax = Min(lmax, lmax3); }

	for(int q = firstActive; q <= lastActive; q++) {
		floatq lmin, lmax;
		COMPUTE(ctx.rayIDir[q], lmin, lmax);
		if(ForAny( lmax >= zero && lmin <= Min(lmax, ctx.distance[q]))) {
			firstActive = q;
			ret = 1;
			break;
		}
	}
	for(int q = lastActive; q >= firstActive; q--) {
		floatq lmin, lmax;
		COMPUTE(ctx.rayIDir[q], lmin, lmax);
		if(ForAny( lmax >= zero && lmin <= Min(lmax, ctx.distance[q]))) {
			lastActive = q;
			ret = 1;
			break;
		}
	}

#undef COMPUTE

	return ret;
}

bool BBox::Test(SecondaryContext &ctx, int &firstActive, int &lastActive) const {
	bool ret = 0;

	Vec3q tmin(min);
	Vec3q tmax(max);
	floatq zero(0.0f);

	for(int q = firstActive; q <= lastActive; q++) {
		Vec3q idir = ctx.rayIDir[q];
		Vec3q origin = ctx.rayOrigin[q];

		floatq l1 = idir.x * (tmin.x - origin.x);
		floatq l2 = idir.x * (tmax.x - origin.x);
		floatq lmin = Min(l1, l2);
		floatq lmax = Max(l1, l2);

		l1 = idir.y * (tmin.y - origin.y);
		l2 = idir.y * (tmax.y - origin.y);
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		l1 = idir.z * (tmin.z - origin.z);
		l2 = idir.z * (tmax.z - origin.z);
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		if(ForAny( lmax >= zero && lmin <= Min(lmax, ctx.distance[q]) )) {
			firstActive = q;
			ret = 1;
			break;
		}
	}
	for(int q = lastActive; q >= firstActive; q--) {
		Vec3q idir = ctx.rayIDir[q];
		Vec3q origin = ctx.rayOrigin[q];

		floatq l1 = idir.x * (tmin.x - origin.x);
		floatq l2 = idir.x * (tmax.x - origin.x);
		floatq lmin = Min(l1, l2);
		floatq lmax = Max(l1, l2);

		l1 = idir.y * (tmin.y - origin.y);
		l2 = idir.y * (tmax.y - origin.y);
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		l1 = idir.z * (tmin.z - origin.z);
		l2 = idir.z * (tmax.z - origin.z);
		lmin = Max(Min(l1, l2), lmin);
		lmax = Min(Max(l1, l2), lmax);

		if(ForAny( lmax >= zero && lmin <= Min(lmax, ctx.distance[q]) )) {
			lastActive = q;
			ret = 1;
			break;
		}
	}

	return ret;
}

template bool BBox::Test<PrimaryContext>(PrimaryContext&, int&, int&) const;
template bool BBox::Test<ShadowContext >(ShadowContext &, int&, int&) const;
