#include "triangle.h"

template <bool sharedOrigin, bool hasMask>
void Triangle::Collide(Context<sharedOrigin, hasMask> &c, int idx, int firstActive, int lastActive) const {
	int count = lastActive - firstActive + 1;

	floatq dets[count], tmuls[count];
	Vec2q uvs[count];
	float tmul; {
		Vec3f tvec0, tvec1;
		const Vec3f nrm = Nrm();
	
		if(sharedOrigin) {
			Vec3f tvec = ExtractN(c.Origin(0), 0) - a;
			tvec0 = (ba ^ tvec) * it0;
			tvec1 = (tvec ^ ca) * it0;
			tmul = -(tvec | nrm);
		}
		
		for(int q = 0; q < count; q++)
		{
			int tq = q + firstActive;
			const Vec3q dir = c.Dir(tq);

			dets[q] = dir | nrm;
			if(sharedOrigin) {
				uvs[q].y = (dir | tvec0);
				uvs[q].x = (dir | tvec1);
			}
			else {
				Vec3q tvec = c.Origin(tq) - Vec3q(a);
				Vec3q tvec0 = Vec3q(ba) ^ tvec;
				Vec3q tvec1 = tvec ^ Vec3q(ca);
				tmuls[q] = -(tvec | nrm);

				uvs[q].y = (dir | tvec0) * it0;
				uvs[q].x = (dir | tvec1) * it0;
			}
		}
	}
		
	for(int q = 0; q < count; q++) {
		int tq = q + firstActive;

		floatq u = uvs[q].x, v = uvs[q].y, det = dets[q];

		floatq duv = det - u - v;
		floatq uvmin = Min(u, Min(v, duv));
		floatq uvmax = Max(u, Max(v, duv));

		f32x4b test = uvmax <= 0.0f || uvmin >= 0.0f;
		if(hasMask) test = test && c.SSEMask(tq);

		if(ForAny(test)) {
			floatq idet = Inv(det);
			floatq dist = idet * (sharedOrigin?tmul : tmuls[q]);
			test = test && dist < c.Distance(tq) && dist > 0.0f;
			c.Distance(tq) = Condition(test, dist, c.Distance(tq));
			c.Object(tq) = Condition(i32x4b(test), i32x4(idx), c.Object(tq));
			c.barycentric[tq] = Condition(test, Vec2q(u, v) * idet, c.barycentric[tq]);
		}
	}
}

bool Triangle::Collide(ShadowContext &c, int firstActive, int lastActive) const {
	int count = lastActive - firstActive + 1;
	bool full = count == c.Size();

	floatq dets[count];
	Vec2q uvs[count];
	floatq tmul;

	{
		Vec3f nrm = Nrm();
		Vec3f tvec = ExtractN(c.Origin(0), 0) - a;
		Vec3f tvec0 = (ba ^ tvec) * it0;
		Vec3f tvec1 = (tvec ^ ca) * it0;
		tmul = -(tvec | nrm);

		for(int q = 0; q < count; q++) {
			int tq = q + firstActive;
			Vec3q dir = c.Dir(tq);
			dets[q] = dir | nrm;
			uvs[q].y = dir | tvec0;
			uvs[q].x = dir | tvec1;
		}
	}

	for(int q = 0; q < count; q++) {
		int tq = q + firstActive;

		floatq det = dets[q], u = uvs[q].x, v = uvs[q].y;
		
		f32x4b test = Min(u, v) >= 0.0f && u + v <= det;
		test = test && tmul > 0.0f && tmul < c.Distance(tq) * det;
		full &= ForAll(test);

		c.Distance(tq) = Condition(test, -constant::inf, c.Distance(tq));
	}

	return full;
}


template void Triangle::Collide<0, 0>(Context<0, 0>&, int, int, int) const;
template void Triangle::Collide<0, 1>(Context<0, 1>&, int, int, int) const;
template void Triangle::Collide<1, 0>(Context<1, 0>&, int, int, int) const;
template void Triangle::Collide<1, 1>(Context<1, 1>&, int, int, int) const;

bool Triangle::TestInterval(const RayInterval &i) const {
	//TODO: min/max origin
	Vec3f nrm = Nrm();

	float det =
		(nrm.x < 0.0f? i.minDir.x : i.maxDir.x) * nrm.x +
		(nrm.y < 0.0f? i.minDir.y : i.maxDir.y) * nrm.y +
		(nrm.z < 0.0f? i.minDir.z : i.maxDir.z) * nrm.z;

	if(det < 0.0f)
		return 1; // dirty hack :) TODO: repair and make it faster

	enum { sharedOrigin = 1 };
	Vec3f c1a, c1b, c2a, c2b;
	if(sharedOrigin) {
		Vec3f tvec = i.minOrigin - a;
		Vec3f c1 = ba ^ tvec, c2 = tvec ^ ca;

		c1a = i.minDir * c1; c1b = i.maxDir * c1;
		c2a = i.minDir * c2; c2b = i.maxDir * c2;
	}
	else {
		Vec3f tvec1 = i.minOrigin - a, tvec2 = i.maxOrigin - a;

		{
			float xa1 = ba.y * tvec1.z, xb1 = ba.z * tvec1.y; float xa2 = ba.y * tvec2.z, xb2 = ba.z * tvec2.y;
			float ya1 = ba.z * tvec1.x, yb1 = ba.x * tvec1.z; float ya2 = ba.z * tvec2.x, yb2 = ba.x * tvec2.z;
			float za1 = ba.x * tvec1.y, zb1 = ba.y * tvec1.x; float za2 = ba.x * tvec2.y, zb2 = ba.y * tvec2.x;
			c1a.x = Min(xa1, xa2) - Max(xb1, xb2); c1b.x = Max(xa1, xa2) - Min(xb1, xb2);
			c1a.y = Min(ya1, ya2) - Max(yb1, yb2); c1b.y = Max(ya1, ya2) - Min(yb1, yb2);
			c1a.z = Min(za1, za2) - Max(zb1, zb2); c1b.z = Max(za1, za2) - Min(zb1, zb2);
		}
		{
			float xa1 = tvec1.y * ca.z, xb1 = tvec1.z * ca.y; float xa2 = tvec2.y * ca.z, xb2 = tvec2.z * ca.y;
			float ya1 = tvec1.z * ca.x, yb1 = tvec1.x * ca.z; float ya2 = tvec2.z * ca.x, yb2 = tvec2.x * ca.z;
			float za1 = tvec1.x * ca.y, zb1 = tvec1.y * ca.x; float za2 = tvec2.x * ca.y, zb2 = tvec2.y * ca.x;
			c2a.x = Min(xa1, xa2) - Max(xb1, xb2); c2b.x = Max(xa1, xa2) - Min(xb1, xb2);
			c2a.y = Min(ya1, ya2) - Max(yb1, yb2); c2b.y = Max(ya1, ya2) - Min(yb1, yb2);
			c2a.z = Min(za1, za2) - Max(zb1, zb2); c2b.z = Max(za1, za2) - Min(zb1, zb2);
		}
		Vec3f t1, t2, t3, t4;
		t1 = i.minDir * c1a; t2 = i.maxDir * c1b; 
		t3 = i.maxDir * c1a; t4 = i.minDir * c1b; 
		c1a = VMin(VMin(t1, t2), VMin(t3, t4)); c1b = VMax(VMax(t1, t2), VMax(t3, t4));
		t1 = i.minDir * c2a; t2 = i.maxDir * c2b;
		t3 = i.maxDir * c2a; t4 = i.minDir * c2b;
		c2a = VMin(VMin(t1, t2), VMin(t3, t4)); c2b = VMax(VMax(t1, t2), VMax(t3, t4));
	}

	float u[2] = {
		Min(c1a.x, c1b.x) + Min(c1a.y, c1b.y) + Min(c1a.z, c1b.z),
		Max(c1a.x, c1b.x) + Max(c1a.y, c1b.y) + Max(c1a.z, c1b.z) };
	float v[2] = {
		Min(c2a.x, c2b.x) + Min(c2a.y, c2b.y) + Min(c2a.z, c2b.z),
		Max(c2a.x, c2b.x) + Max(c2a.y, c2b.y) + Max(c2a.z, c2b.z) };
	
	return Min(u[1], v[1]) >= 0.0f && u[0] + v[0] <= det * t0;
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

	Vec3q tvec(rays.origin - a);
	floatq u = rays.dir | (Vec3q(ba) ^ tvec);
	floatq v = rays.dir | (tvec ^ Vec3q(ca));

	floatq duv = det * t0 - u - v;
	floatq sign = Condition(det < 0.0f, -1.0f, 1.0f);

	return ForAny(u * sign > 0.0f) && ForAny(v * sign > 0.0f) && ForAny(duv * sign > 0.0f);

	//TODO: SAT?
}
