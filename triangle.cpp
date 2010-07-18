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

template <bool sharedOrigin, bool hasMask>
void Triangle::Collide(Context<sharedOrigin, hasMask> &c, int idx, int firstActive, int lastActive) const {
	Vec3f nrm = Nrm(), tvec0, tvec1;
	float tmul;

	if(hasMask && !sharedOrigin)
		exit(0); //TODO

	if(sharedOrigin) {
		Vec3f tvec = ExtractN(c.Origin(0), 0) - a;
		tvec0 = (ba ^ tvec) * it0;
		tvec1 = (tvec ^ ca) * it0;
		tmul = -(tvec | nrm);
	}

	int count = lastActive - firstActive + 1;
	for(int q = 0; q < count; q++) {
		int tq = q + firstActive;

		const Vec3q dir = c.Dir(tq);
		floatq idet = Inv(dir.x * nrm.x + dir.y * nrm.y + dir.z * nrm.z);

		floatq u, v, dist;
		if(sharedOrigin) {
			dist = idet * tmul;
			v = (dir.x * tvec0.x + dir.y * tvec0.y + dir.z * tvec0.z) * idet;
			u = (dir.x * tvec1.x + dir.y * tvec1.y + dir.z * tvec1.z) * idet;
		}
		else {
			Vec3q tvec = c.Origin(tq) - Vec3q(a);
			dist = -(tvec | nrm) * idet;
			Vec3q tvec0 = Vec3q(ba) ^ tvec;
			Vec3q tvec1 = tvec ^ Vec3q(ca);

			idet *= it0;
			v = (dir | tvec0) * idet;
			u = (dir | tvec1) * idet;
		}

		f32x4b test = Min(u, v) >= 0.0f && u + v <= floatq(1.0f);
		test = test && dist >= floatq(0.0f) && dist < c.Distance(tq);

		c.Distance(tq) = Condition(test, dist, c.Distance(tq));
		c.Object(tq) = Condition(i32x4b(test), i32x4(idx), c.Object(tq));
		c.barycentric[tq] = Condition(test, Vec2q(u, v), c.barycentric[tq]);
	}
}

template <bool sharedOrigin, bool hasMask>
int Triangle::CollideShadow(Context<sharedOrigin, hasMask> &c, int firstActive, int lastActive) const {
	Vec3f nrm = Nrm();
	Vec3q ta(a);

	Vec3q sharedTVec;
	if(sharedOrigin)
		sharedTVec = c.Origin(0) - ta;

	for(int q = firstActive; q <= lastActive; q++) {
		floatq det  = c.Dir(q) | nrm;
		Vec3q  tvec = sharedOrigin? sharedTVec : c.Origin(q) - ta;

		floatq u    = c.Dir(q) | (Vec3q(ba) ^ tvec);
		floatq v    = c.Dir(q) | (tvec ^ Vec3q(ca));
		f32x4b test = Min(u, v) >= 0.0f && u + v <= det * floatq(t0);

		floatq dist = -(tvec | nrm) / det;
		test = test && dist > floatq(0.0f) && dist < c.Distance(q);

		c.Distance(q) = Condition(test, dist, c.Distance(q));
		if(hasMask) c.MaskPtr()[q] &= ~ForWhich(test);
	}

	return 0;
}


template void Triangle::Collide<0, 0>(Context<0, 0>&, int, int, int) const;
template void Triangle::Collide<0, 1>(Context<0, 1>&, int, int, int) const;
template void Triangle::Collide<1, 0>(Context<1, 0>&, int, int, int) const;
template void Triangle::Collide<1, 1>(Context<1, 1>&, int, int, int) const;

template int Triangle::CollideShadow<0, 0>(Context<0, 0>&, int, int) const;
template int Triangle::CollideShadow<0, 1>(Context<0, 1>&, int, int) const;
template int Triangle::CollideShadow<1, 0>(Context<1, 0>&, int, int) const;
template int Triangle::CollideShadow<1, 1>(Context<1, 1>&, int, int) const;

bool Triangle::TestInterval(const RayInterval &i) const {
	//TODO: min/max origin
	Vec3f nrm = Nrm();

	float det =
		(nrm.x < 0.0f? i.minDir.x : i.maxDir.x) * nrm.x +
		(nrm.y < 0.0f? i.minDir.y : i.maxDir.y) * nrm.y +
		(nrm.z < 0.0f? i.minDir.z : i.maxDir.z) * nrm.z;
	if(det < 0.0f)
		return 0;

	Vec3f tvec = i.minOrigin - a;
	Vec3f c1 = ba ^ tvec, c2 = tvec ^ ca;
	Vec3f c1a = i.minDir * c1, c1b = i.maxDir * c1;
	Vec3f c2a = i.minDir * c2, c2b = i.maxDir * c2;

	float u[2] = {
		Min(c1a.x, c1b.x) + Min(c1a.y, c1b.y) + Min(c1a.z, c1b.z),
		Max(c1a.x, c1b.x) + Max(c1a.y, c1b.y) + Max(c1a.z, c1b.z) };
	float v[2] = {
		Min(c2a.x, c2b.x) + Min(c2a.y, c2b.y) + Min(c2a.z, c2b.z),
		Max(c2a.x, c2b.x) + Max(c2a.y, c2b.y) + Max(c2a.z, c2b.z) };
	
	return Min(u[1], v[1]) >= 0.0f && u[0] + v[0] <= det * t0;
/*	// TODO: zleeee
	Vec3<Interval> tvec(
		Interval(i.minOrigin.x, i.maxOrigin.x) - a.x,
		Interval(i.minOrigin.y, i.maxOrigin.y) - a.y,
		Interval(i.minOrigin.z, i.maxOrigin.z) - a.z);

	Vec3<Interval> c1 = Vec3<Interval>(ba) ^ tvec, c2 = tvec ^ Vec3<Interval>(ca);
	Vec3<Interval> dir(
			Interval(i.minDir.x, i.maxDir.x), Interval(i.minDir.y, i.maxDir.y), Interval(i.minDir.z, i.maxDir.z) );

	InputAssert(t0 >= 0);
	Interval u = c1 | dir, v = c2 | dir;

	return Min(u.max, v.max) >= 0.0f && u.min + v.min <= det * t0; */
}

bool Triangle::TestFrustum(const Frustum &frustum) const {
	Vec3f a = P1(), b = P2(), c = P3();

	for(int p = 0; p < 4; p++) {
		const Plane &plane = frustum.planes[p];

		if(Max(plane.normal | a, Max(plane.normal | b, plane.normal | c)) < plane.distance)
			return 0;
	}
	return 1;
}

bool Triangle::TestCornerRays(const CornerRays &rays) const {
	Vec3f nrm = Nrm();
	floatq det = rays.dir | nrm;
	if(ForAll(det < 0.0f))
		return 0;

	Vec3q tvec(rays.origin - a);
	floatq u = rays.dir | (Vec3q(ba) ^ tvec);
	floatq v = rays.dir | (tvec ^ Vec3q(ca));

	if(ForAll(u < 0.0f) || ForAll(v < 0.0f) || ForAll(u + v > det * floatq(t0)))
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
