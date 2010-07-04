#include "triangle.h"

namespace {

	struct Interval {
		Interval(float min, float max) :min(min), max(max) { }
		Interval(float v) :min(v), max(v) { }
		Interval() { }

		const Interval operator+(const Interval rhs) const {
			return Interval(min + rhs.min, max + rhs.max);
		}
		const Interval operator-(const Interval rhs) const {
			return Interval(min - rhs.max, max - rhs.min);
		}
		const Interval operator*(const Interval rhs) const {
			float a = min * rhs.min;
			float b = min * rhs.max;
			float c = max * rhs.min;
			float d = max * rhs.max;

			return Interval(Min(Min(a, b), Min(c, d)), Max(Max(a, b), Max(c, d)));
		}
		const Interval operator-() const {
			return Interval(-max, -min);
		}

		float min, max;
	};

}

bool Triangle::TestInterval(const RayInterval &i) const {
	Vec3f nrm = Nrm();

	float det =
		(nrm.x < 0.0f? i.minDir.x : i.maxDir.x) * nrm.x +
		(nrm.y < 0.0f? i.minDir.y : i.maxDir.y) * nrm.y +
		(nrm.z < 0.0f? i.minDir.z : i.maxDir.z) * nrm.z;
	if(det < 0.0f)
		return 0;

	Vec3f tvec = i.minOrigin - a;
	Vec3f c1 = ba ^ tvec, c2 = tvec ^ ca;
	float u[2] = {
		(c1.x < 0.0f? i.maxDir.x : i.minDir.x) * c1.x + (c1.y < 0.0f? i.maxDir.y : i.minDir.y) * c1.y + (c1.z < 0.0f? i.maxDir.z : i.minDir.z) * c1.z,
		(c1.x < 0.0f? i.minDir.x : i.maxDir.x) * c1.x + (c1.y < 0.0f? i.minDir.y : i.maxDir.y) * c1.y + (c1.z < 0.0f? i.minDir.z : i.maxDir.z) * c1.z };
	float v[2] = {
		(c2.x < 0.0f? i.maxDir.x : i.minDir.x) * c2.x + (c2.y < 0.0f? i.maxDir.y : i.minDir.y) * c2.y + (c2.z < 0.0f? i.maxDir.z : i.minDir.z) * c2.z,
		(c2.x < 0.0f? i.minDir.x : i.maxDir.x) * c2.x + (c2.y < 0.0f? i.minDir.y : i.maxDir.y) * c2.y + (c2.z < 0.0f? i.minDir.z : i.maxDir.z) * c2.z };
	
	return Min(u[1], v[1]) >= 0.0f && u[0] + v[0] <= det * ca.t0;

/*	// TODO: zleeee
	Vec3<Interval> tvec(
		Interval(i.minOrigin.x, i.maxOrigin.x) - a.x,
		Interval(i.minOrigin.y, i.maxOrigin.y) - a.y,
		Interval(i.minOrigin.z, i.maxOrigin.z) - a.z);

	Vec3<Interval> c1 = Vec3<Interval>(ba) ^ tvec, c2 = tvec ^ Vec3<Interval>(ca);
	Vec3<Interval> dir(
			Interval(i.minDir.x, i.maxDir.x), Interval(i.minDir.y, i.maxDir.y), Interval(i.minDir.z, i.maxDir.z) );

	Interval u = c1 | dir, v = c2 | dir;

	return Min(u.max, v.max) >= 0.0f && u.min + v.min <= det * ca.t0; */
}

bool Triangle::TestCornerRays(const CornerRays &rays) const {
	Vec3f nrm = Nrm();
	floatq det = rays.dir | nrm;
	if(ForAll(det < 0.0f))
		return 0;

	Vec3q tvec(rays.origin - a);
	floatq u = rays.dir | (Vec3q(ba) ^ tvec);
	floatq v = rays.dir | (tvec ^ Vec3q(ca));

	if(ForAll(u < 0.0f) || ForAll(v < 0.0f) || ForAll(u + v > det * floatq(ca.t0)))
		return 0;

	return 1;
	// TODO poprawic...

/*	u = _mm_shuffle_ps(u.m, u.m, (0 << 0) | (1 << 2) | (3 << 4) | (2 << 6));
	v = _mm_shuffle_ps(v.m, v.m, (0 << 0) | (1 << 2) | (3 << 4) | (2 << 6));

	floatq su = _mm_shuffle_ps(u.m, u.m, (1 << 0) | (2 << 2) | (3 << 4) | (0 << 6));
	floatq sv = _mm_shuffle_ps(v.m, v.m, (1 << 0) | (2 << 2) | (3 << 4) | (0 << 6));

	floatq ssu = _mm_shuffle_ps(su.m, su.m, (1 << 0) | (2 << 2) | (3 << 4) | (0 << 6));
	floatq ssv = _mm_shuffle_ps(sv.m, sv.m, (1 << 0) | (2 << 2) | (3 << 4) | (0 << 6));

	Vec2q cnrm((sv - v), -(su - u));
	cnrm *= RSqrt(cnrm | cnrm);

	floatq dist = u * cnrm.x + v * cnrm.y;
	if(ForAny(ssu * cnrm.x + ssv * cnrm.y - dist < 0.0f))
		return 1;

	floatq side0 = cnrm.x * 0.0f + cnrm.y * 0.0f - dist;
	floatq side1 = cnrm.x * 1.0f + cnrm.y * 0.0f - dist;
	floatq side2 = cnrm.x * 0.0f + cnrm.y * 1.0f - dist;

	floatq du = su - u, dv = sv - v;
	floatq ddu = _mm_shuffle_ps(du.m, du.m, (1 << 0) | (2 << 2) | (3 << 4) | (0 << 6));
	floatq ddv = _mm_shuffle_ps(dv.m, dv.m, (1 << 0) | (2 << 2) | (3 << 4) | (0 << 6));
	
	bool isConvex =
		CountMaskBits( ForWhich(_mm_xor_ps((du < 0.0f).m, (ddu < 0.0f).m)) ) <= 2 &&
		CountMaskBits( ForWhich(_mm_xor_ps((dv < 0.0f).m, (ddv < 0.0f).m)) ) <= 2;
//	du/=Abs(du); dv/=Abs(dv); ddu/=Abs(ddu); ddv/=Abs(ddv);
//	std::cout << isConvex << '\n' << du << ' ' << dv << '\n' << ddu << ' ' << ddv << "\n\n";

	bool ret = !( isConvex && ForAny(side0 < 0.0f && side1 < 0.0f && side2 < 0.0f) );
	std::cout << ret;
	return ret; */
}
