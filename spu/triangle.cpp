#include "spu/triangle.h"
#include "spu/trace.h"
#include "spu/stats.h"

template <class Context>
inline void Triangle::Prepare(const Context &ctx, Vec3q &tnrm, Vec3q &tvec0, Vec3q &tvec1, floatq &tmul) const {
	tnrm = Vec3q(plane.x, plane.y, plane.z);
	Vec3q tvec = ctx.rayOrigin - Vec3q(a);
	tvec0 = (Vec3q(ba) ^ tvec) * floatq(it0);
	tvec1 = (tvec ^ Vec3q(ca)) * floatq(it0);
	tmul = -(tvec | tnrm);
}
	
inline void Triangle::Prepare(const SecondaryContext &ctx, Vec3q &tnrm,
							Vec3q &ta, Vec3q &tba, Vec3q &tca, floatq &tit0) const {
	tnrm = Vec3q(plane.x, plane.y, plane.z);
	ta = Vec3q(a);
	tba = Vec3q(ba);
	tca = Vec3q(ca);
	tit0 = it0;
}

void MultiCollide(const Triangle &tri0, const Triangle &tri1, const Triangle &tri2, const Triangle &tri3,
					PrimaryContext &ctx, int sidx, int firstA, int lastA) {
	//TODO: ulozyc odpowiednio (wektorowo) i liczyc test RayInterval - 4 trojkaty
	Vec3q tnrm[4], tvec0[4], tvec1[4];
	floatq tmul[4], zero(0.0f), one(1.0f);
	i32x4 idx[4] = { sidx, sidx + 1, sidx + 2, sidx + 3 };

/*	{
		Vec3q ttnrm(	floatq(tri0.plane.x, tri1.plane.x, tri2.plane.x, tri3.plane.x),
						floatq(tri0.plane.y, tri1.plane.y, tri2.plane.y, tri3.plane.y),
						floatq(tri0.plane.z, tri1.plane.z, tri2.plane.z, tri3.plane.z) );
		Vec3q tvec = ctx.rayOrigin - Vec3q(
				floatq(tri0.a.x, tri1.a.x, tri2.a.x, tri3.a.x),
				floatq(tri0.a.y, tri1.a.y, tri2.a.y, tri3.a.y),
				floatq(tri0.a.z, tri1.a.z, tri2.a.z, tri3.a.z) );
		Vec3q tba(	floatq(tri0.ba.x, tri1.ba.x, tri2.ba.x, tri3.ba.x),
					floatq(tri0.ba.y, tri1.ba.y, tri2.ba.y, tri3.ba.y),
					floatq(tri0.ba.z, tri1.ba.z, tri2.ba.z, tri3.ba.z) );
		Vec3q tca(	floatq(tri0.ca.x, tri1.ca.x, tri2.ca.x, tri3.ca.x),
					floatq(tri0.ca.y, tri1.ca.y, tri2.ca.y, tri3.ca.y),
					floatq(tri0.ca.z, tri1.ca.z, tri2.ca.z, tri3.ca.z) );
		floatq tit0(tri0.it0, tri1.it0, tri2.it0, tri3.it0);
		Vec3q ttvec0 = (tba ^ tvec) * tit0;
		Vec3q ttvec1 = (tvec ^ tca) * tit0;
		floatq ttmul = -(tvec | ttnrm);
		for(int k = 0; k < 4; k++) {
			tnrm[k] = Vec3q(ttnrm.x[k], ttnrm.y[k], ttnrm.z[k]);
			tvec0[k] = Vec3q(ttvec0.x[k], ttvec0.y[k], ttvec0.z[k]);
			tvec1[k] = Vec3q(ttvec1.x[k], ttvec1.y[k], ttvec1.z[k]);
			tmul[k] = ttmul[k];
		}
	} */

	tri0.Prepare(ctx, tnrm[0], tvec0[0], tvec1[0], tmul[0]);
	tri1.Prepare(ctx, tnrm[1], tvec0[1], tvec1[1], tmul[1]);
	tri2.Prepare(ctx, tnrm[2], tvec0[2], tvec1[2], tmul[2]);
	tri3.Prepare(ctx, tnrm[3], tvec0[3], tvec1[3], tmul[3]);

	for(int q = firstA; q <= lastA; q++) {
		Vec3q dir = ctx.rayDir[q];
		floatq distance = ctx.distance[q];
		i32x4 triIds = ctx.triIds[q];

		floatq idet[4] = { Inv(dir | tnrm[0]), Inv(dir | tnrm[1]), Inv(dir | tnrm[2]), Inv(dir | tnrm[3]) };
		floatq dist[4] = { idet[0] * tmul[0], idet[1] * tmul[1], idet[2] * tmul[2], idet[3] * tmul[3] };
		Vec2q barycentric = ctx.barycentric[q];
		Vec3q normals = ctx.normals[q];

		floatq v[4] = {
			(dir | tvec0[0]) * idet[0],
			(dir | tvec0[1]) * idet[1],
			(dir | tvec0[2]) * idet[2],
			(dir | tvec0[3]) * idet[3],
		};
		floatq u[4] = {
			(dir | tvec1[0]) * idet[0],
			(dir | tvec1[1]) * idet[1],
			(dir | tvec1[2]) * idet[2],
			(dir | tvec1[3]) * idet[3],
		};

		f32x4b test[4] = {
			Min(u[0], v[0]) >= zero && u[0] + v[0] <= one,
			Min(u[1], v[1]) >= zero && u[1] + v[1] <= one,
			Min(u[2], v[2]) >= zero && u[2] + v[2] <= one,
			Min(u[3], v[3]) >= zero && u[3] + v[3] <= one,
		};
		
		
		test[0] = test[0] && idet[0] > zero && dist[0] >= zero;
		test[1] = test[1] && idet[1] > zero && dist[1] >= zero;
		test[2] = test[2] && idet[2] > zero && dist[2] >= zero;
		test[3] = test[3] && idet[3] > zero && dist[3] >= zero;
		
		f32x4 minDist = distance;
		minDist = Condition(test[0] && dist[0] < minDist, dist[0], minDist);
		minDist = Condition(test[1] && dist[1] < minDist, dist[1], minDist);
		minDist = Condition(test[2] && dist[2] < minDist, dist[2], minDist);
		minDist = Condition(test[3] && dist[3] < minDist, dist[3], minDist);

		test[0] = test[0] && dist[0] <= minDist;
		test[1] = test[1] && dist[1] <= minDist;
		test[2] = test[2] && dist[2] <= minDist;
		test[3] = test[3] && dist[3] <= minDist;

		ctx.distance[q] = minDist;
		ctx.normals[q] =
			Condition(test[0], tnrm[0],
			Condition(test[1], tnrm[1],
			Condition(test[2], tnrm[2],
			Condition(test[3], tnrm[3], normals))));
		ctx.triIds[q] =
			Condition(i32x4b(test[0]), idx[0],
			Condition(i32x4b(test[1]), idx[1],
			Condition(i32x4b(test[2]), idx[2],
			Condition(i32x4b(test[3]), idx[3], triIds))));
		ctx.barycentric[q] =
			Condition(test[0], Vec2q(u[0], v[0]),
			Condition(test[1], Vec2q(u[1], v[1]),
			Condition(test[2], Vec2q(u[2], v[2]),
			Condition(test[3], Vec2q(u[3], v[3]), barycentric))));
	}
}

void MultiCollide(const Triangle &tri0, const Triangle &tri1, const Triangle &tri2, const Triangle &tri3,
					ShadowContext &ctx, int sidx, int firstA, int lastA) {
	//TODO: ulozyc odpowiednio (wektorowo) i liczyc test RayInterval - 4 trojkaty
	Vec3q tnrm[4], tvec0[4], tvec1[4];
	floatq tmul[4], zero(0.0f), one(1.0f);

	tri0.Prepare(ctx, tnrm[0], tvec0[0], tvec1[0], tmul[0]);
	tri1.Prepare(ctx, tnrm[1], tvec0[1], tvec1[1], tmul[1]);
	tri2.Prepare(ctx, tnrm[2], tvec0[2], tvec1[2], tmul[2]);
	tri3.Prepare(ctx, tnrm[3], tvec0[3], tvec1[3], tmul[3]);

	for(int q = firstA; q <= lastA; q++) {
		Vec3q dir = ctx.rayDir[q];
		floatq distance = ctx.distance[q];

		floatq idet[4] = { Inv(dir | tnrm[0]), Inv(dir | tnrm[1]), Inv(dir | tnrm[2]), Inv(dir | tnrm[3]) };
		floatq dist[4] = { idet[0] * tmul[0], idet[1] * tmul[1], idet[2] * tmul[2], idet[3] * tmul[3] };

		floatq v[4] = {
			(dir | tvec0[0]) * idet[0],
			(dir | tvec0[1]) * idet[1],
			(dir | tvec0[2]) * idet[2],
			(dir | tvec0[3]) * idet[3],
		};
		floatq u[4] = {
			(dir | tvec1[0]) * idet[0],
			(dir | tvec1[1]) * idet[1],
			(dir | tvec1[2]) * idet[2],
			(dir | tvec1[3]) * idet[3],
		};

		f32x4b test[4] = {
			Min(u[0], v[0]) >= zero && u[0] + v[0] <= one,
			Min(u[1], v[1]) >= zero && u[1] + v[1] <= one,
			Min(u[2], v[2]) >= zero && u[2] + v[2] <= one,
			Min(u[3], v[3]) >= zero && u[3] + v[3] <= one,
		};
		
		
		test[0] = test[0] && idet[0] > zero && dist[0] >= zero;
		test[1] = test[1] && idet[1] > zero && dist[1] >= zero;
		test[2] = test[2] && idet[2] > zero && dist[2] >= zero;
		test[3] = test[3] && idet[3] > zero && dist[3] >= zero;
		
		f32x4 minDist = distance;
		minDist = Condition(test[0] && dist[0] < minDist, dist[0], minDist);
		minDist = Condition(test[1] && dist[1] < minDist, dist[1], minDist);
		minDist = Condition(test[2] && dist[2] < minDist, dist[2], minDist);
		minDist = Condition(test[3] && dist[3] < minDist, dist[3], minDist);

		test[0] = test[0] && dist[0] <= minDist;
		test[1] = test[1] && dist[1] <= minDist;
		test[2] = test[2] && dist[2] <= minDist;
		test[3] = test[3] && dist[3] <= minDist;

		ctx.distance[q] = minDist;
	}
}

void MultiCollide(const Triangle &tri0, const Triangle &tri1, const Triangle &tri2, const Triangle &tri3,
					SecondaryContext &ctx, int sidx, int firstA, int lastA) {
	//TODO: ulozyc odpowiednio (wektorowo) i liczyc test RayInterval - 4 trojkaty
	Vec3q tnrm[4], ta[4], tca[4], tba[4];
	floatq tit0[4], zero(0.0f), one(1.0f);
	i32x4 idx(sidx + 0, sidx + 1, sidx + 2, sidx + 3);

	tri0.Prepare(ctx, tnrm[0], ta[0], tba[0], tca[0], tit0[0]);
	tri1.Prepare(ctx, tnrm[1], ta[1], tba[1], tca[1], tit0[1]);
	tri2.Prepare(ctx, tnrm[2], ta[2], tba[2], tca[2], tit0[2]);
	tri3.Prepare(ctx, tnrm[3], ta[3], tba[3], tca[3], tit0[3]);

	for(int q = firstA; q <= lastA; q++) {
		Vec3q dir = ctx.rayDir[q];
		Vec3q origin = ctx.rayOrigin[q];
		floatq distance = ctx.distance[q];
		i32x4 triIds = ctx.triIds[q];

		floatq idet[4] = { Inv(dir | tnrm[0]), Inv(dir | tnrm[1]), Inv(dir | tnrm[2]), Inv(dir | tnrm[3]) };
		Vec3q tvec[4] = { origin - ta[0], origin - ta[1], origin - ta[2], origin - ta[3] };
		floatq dist[4] = {
			-idet[0] * (tvec[0] | tnrm[0]),
			-idet[1] * (tvec[1] | tnrm[1]),
			-idet[2] * (tvec[2] | tnrm[2]),
			-idet[3] * (tvec[3] | tnrm[3]) };
		Vec2q barycentric = ctx.barycentric[q];
		Vec3q normals = ctx.normals[q];

		Vec3q tvec0[4] = { tba[0] ^ tvec[0], tba[1] ^ tvec[1], tba[2] ^ tvec[2], tba[3] ^ tvec[3] };
		Vec3q tvec1[4] = { tvec[0] ^ tca[0], tvec[1] ^ tca[1], tvec[2] ^ tca[2], tvec[3] ^ tca[3] };
		idet[0] *= tit0[0];
		idet[1] *= tit0[1];
		idet[2] *= tit0[2];
		idet[3] *= tit0[3];

		floatq v[4] = {
			(dir | tvec0[0]) * idet[0],
			(dir | tvec0[1]) * idet[1],
			(dir | tvec0[2]) * idet[2],
			(dir | tvec0[3]) * idet[3],
		};
		floatq u[4] = {
			(dir | tvec1[0]) * idet[0],
			(dir | tvec1[1]) * idet[1],
			(dir | tvec1[2]) * idet[2],
			(dir | tvec1[3]) * idet[3],
		};

		f32x4b test[4] = {
			Min(u[0], v[0]) >= zero && u[0] + v[0] <= one,
			Min(u[1], v[1]) >= zero && u[1] + v[1] <= one,
			Min(u[2], v[2]) >= zero && u[2] + v[2] <= one,
			Min(u[3], v[3]) >= zero && u[3] + v[3] <= one,
		};
		
		
		test[0] = test[0] && idet[0] > zero && dist[0] >= zero;
		test[1] = test[1] && idet[1] > zero && dist[1] >= zero;
		test[2] = test[2] && idet[2] > zero && dist[2] >= zero;
		test[3] = test[3] && idet[3] > zero && dist[3] >= zero;
		
		f32x4 minDist = distance;
		minDist = Condition(test[0] && dist[0] < minDist, dist[0], minDist);
		minDist = Condition(test[1] && dist[1] < minDist, dist[1], minDist);
		minDist = Condition(test[2] && dist[2] < minDist, dist[2], minDist);
		minDist = Condition(test[3] && dist[3] < minDist, dist[3], minDist);

		test[0] = test[0] && dist[0] <= minDist;
		test[1] = test[1] && dist[1] <= minDist;
		test[2] = test[2] && dist[2] <= minDist;
		test[3] = test[3] && dist[3] <= minDist;

		ctx.distance[q] = minDist;
		ctx.normals[q] =
			Condition(test[0], tnrm[0],
			Condition(test[1], tnrm[1],
			Condition(test[2], tnrm[2],
			Condition(test[3], tnrm[3], normals))));
		ctx.triIds[q] =
			Condition(i32x4b(test[0]), idx[0],
			Condition(i32x4b(test[1]), idx[1],
			Condition(i32x4b(test[2]), idx[2],
			Condition(i32x4b(test[3]), idx[3], triIds))));
		ctx.barycentric[q] =
			Condition(test[0], Vec2q(u[0], v[0]),
			Condition(test[1], Vec2q(u[1], v[1]),
			Condition(test[2], Vec2q(u[2], v[2]),
			Condition(test[3], Vec2q(u[3], v[3]), barycentric))));
	}
}
void Triangle::Collide(PrimaryContext &ctx, int idx, int first, int last) const { //40%
	Vec3q tnrm(plane.x, plane.y, plane.z);
	Vec3q tvec0, tvec1;
	floatq tmul; {
		Vec3q ta(a), tca(ca), tba(ba);

		Vec3q tvec = ctx.rayOrigin - ta;
		tvec0 = (tba ^ tvec) * floatq(it0);
		tvec1 = (tvec ^ tca) * floatq(it0);
		tmul = -(tvec | Vec3q(plane.x, plane.y, plane.z));
	}
	floatq zero(0.0f), one(1.0f);

	//TODO: nany i denormalne z koncowek moga wplywac na wydajnosc
	while(first <= last) {
		int q = first; first += 4;

		Vec3q dir[4] = { ctx.rayDir[q + 0], ctx.rayDir[q + 1], ctx.rayDir[q + 2], ctx.rayDir[q + 3] };
		i32x4 triIds[4] = { ctx.triIds[q + 0], ctx.triIds[q + 1], ctx.triIds[q + 2], ctx.triIds[q + 3] };
		f32x4 distance[4] = { ctx.distance[q + 0], ctx.distance[q + 1], ctx.distance[q + 2], ctx.distance[q + 3] };

		floatq idet[4] = { Inv(dir[0] | tnrm), Inv(dir[1] | tnrm), Inv(dir[2] | tnrm), Inv(dir[3] | tnrm) };
		f32x4b test[4] = { idet[0] > zero, idet[1] > zero, idet[2] > zero, idet[3] > zero };
		floatq dist[4] = { idet[0] * tmul, idet[1] * tmul, idet[2] * tmul, idet[3] * tmul };

		floatq v[4] = {
			(dir[0] | tvec0) * idet[0], (dir[1] | tvec0) * idet[1],
			(dir[2] | tvec0) * idet[2], (dir[3] | tvec0) * idet[3] };
		Vec2q barycentric[4] = { ctx.barycentric[q + 0], ctx.barycentric[q + 1], ctx.barycentric[q + 2], ctx.barycentric[q + 3] };

		floatq u[4] = {
			(dir[0] | tvec1) * idet[0], (dir[1] | tvec1) * idet[1],
			(dir[2] | tvec1) * idet[2], (dir[3] | tvec1) * idet[3] };
		Vec3q normals[4] = { ctx.normals[q + 0], ctx.normals[q + 1], ctx.normals[q + 2], ctx.normals[q + 3] }; 

		test[0] = ((test[0] && u[0] > zero) && (v[0] > zero && u[0] + v[0] < one))
				&& (dist[0] > zero && dist[0] < distance[0]);
		test[1] = ((test[1] && u[1] > zero) && (v[1] > zero && u[1] + v[1] < one))
				&& (dist[1] > zero && dist[1] < distance[1]);
		test[2] = ((test[2] && u[2] > zero) && (v[2] > zero && u[2] + v[2] < one))
				&& (dist[2] > zero && dist[2] < distance[2]);
		test[3] = ((test[3] && u[3] > zero) && (v[3] > zero && u[3] + v[3] < one))
				&& (dist[3] > zero && dist[3] < distance[3]);

		ctx.distance[q + 0] = Condition(test[0], dist[0], distance[0]);
		ctx.distance[q + 1] = Condition(test[1], dist[1], distance[1]);
		ctx.distance[q + 2] = Condition(test[2], dist[2], distance[2]);
		ctx.distance[q + 3] = Condition(test[3], dist[3], distance[3]);

		ctx.normals[q + 0] = Condition(test[0], tnrm, normals[0]);
		ctx.normals[q + 1] = Condition(test[1], tnrm, normals[1]);
		ctx.normals[q + 2] = Condition(test[2], tnrm, normals[2]);
		ctx.normals[q + 3] = Condition(test[3], tnrm, normals[3]);

		ctx.triIds[q + 0] = Condition(i32x4b(test[0]), idx, triIds[0]);
		ctx.triIds[q + 1] = Condition(i32x4b(test[1]), idx, triIds[1]);
		ctx.triIds[q + 2] = Condition(i32x4b(test[2]), idx, triIds[2]);
		ctx.triIds[q + 3] = Condition(i32x4b(test[3]), idx, triIds[3]);

		ctx.barycentric[q + 0] = Condition(test[0], Vec2q(u[0], v[0]), barycentric[0]);
		ctx.barycentric[q + 1] = Condition(test[1], Vec2q(u[1], v[1]), barycentric[1]);
		ctx.barycentric[q + 2] = Condition(test[2], Vec2q(u[2], v[2]), barycentric[2]);
		ctx.barycentric[q + 3] = Condition(test[3], Vec2q(u[3], v[3]), barycentric[3]);
	}
	if(0) while(first <= last) {
		int q = first++;

		Vec3q dir = ctx.rayDir[q];
		floatq distance = ctx.distance[q];
		i32x4 triIds = ctx.triIds[q];

		floatq idet = Inv(dir | tnrm);

		floatq dist = idet * tmul;
		Vec2q barycentric = ctx.barycentric[q];
		Vec3q normals = ctx.normals[q];

		floatq v = (dir | tvec0) * idet;
		floatq u = (dir | tvec1) * idet;

		f32x4b test = Min(u, v) >= zero && u + v <= one;
		test = test && idet > zero && dist >= zero && dist < distance;

		ctx.distance[q] = Condition(test, dist, distance);
		ctx.normals[q] = Condition(test, tnrm, normals);
		ctx.triIds[q] = Condition(i32x4b(test), idx, triIds);
		ctx.barycentric[q] = Condition(test, Vec2q(u, v), barycentric);
	}
}

void Triangle::Collide(ShadowContext &ctx, int idx, int first, int last) const { //40%
	Vec3q tnrm(plane.x, plane.y, plane.z);
	Vec3q tvec0, tvec1;
	floatq tmul; {
		Vec3q ta(a), tca(ca), tba(ba);

		Vec3q tvec = ctx.rayOrigin - ta;
		tvec0 = (tba ^ tvec) * floatq(it0);
		tvec1 = (tvec ^ tca) * floatq(it0);
		tmul = -(tvec | Vec3q(plane.x, plane.y, plane.z));
	}
	floatq zero(0.0f), one(1.0f);

	while(first <= last) {
		int q = first; first += 4;

		Vec3q dir[4] = { ctx.rayDir[q + 0], ctx.rayDir[q + 1], ctx.rayDir[q + 2], ctx.rayDir[q + 3] };
		f32x4 distance[4] = { ctx.distance[q + 0], ctx.distance[q + 1], ctx.distance[q + 2], ctx.distance[q + 3] };

		floatq det[4] = { dir[0] | tnrm, dir[1] | tnrm, dir[2] | tnrm, dir[3] | tnrm };
		floatq idet[4] = { Inv(det[0]), Inv(det[1]), Inv(det[2]), Inv(det[3]) }; 
		floatq dist[4] = { idet[0] * tmul, idet[1] * tmul, idet[2] * tmul, idet[3] * tmul };

		f32x4b test[4] = {
			dist[0] >= zero && dist[0] < distance[0]/* && det[0] >= zero*/,
			dist[1] >= zero && dist[1] < distance[1]/* && det[1] >= zero*/,
			dist[2] >= zero && dist[2] < distance[2]/* && det[2] >= zero*/,
			dist[3] >= zero && dist[3] < distance[3]/* && det[3] >= zero*/ };

		floatq tv[4] = { dir[0] | tvec0, dir[1] | tvec0, dir[2] | tvec0, dir[3] | tvec0 };
		floatq tu[4] = { dir[0] | tvec1, dir[1] | tvec1, dir[2] | tvec1, dir[3] | tvec1 };

		test[0] = (test[0] && tu[0] >= zero) && (tv[0] >= zero && tu[0] + tv[0] <= det[0]);
		test[1] = (test[1] && tu[1] >= zero) && (tv[1] >= zero && tu[1] + tv[1] <= det[1]);
		test[2] = (test[2] && tu[2] >= zero) && (tv[2] >= zero && tu[2] + tv[2] <= det[2]);
		test[3] = (test[3] && tu[3] >= zero) && (tv[3] >= zero && tu[3] + tv[3] <= det[3]);

		ctx.distance[q + 0] = Condition(test[0], dist[0], distance[0]);
		ctx.distance[q + 1] = Condition(test[1], dist[1], distance[1]);
		ctx.distance[q + 2] = Condition(test[2], dist[2], distance[2]);
		ctx.distance[q + 3] = Condition(test[3], dist[3], distance[3]);
	}
	if(0) while(first <= last) {
		int q = first++;
		Vec3q dir = ctx.rayDir[q];
		floatq distance = ctx.distance[q];

		floatq det = dir | tnrm;
		floatq idet = Inv(det);

		floatq dist = idet * tmul;
		floatq v = (dir | tvec0);
		floatq u = (dir | tvec1);

		f32x4b test = u >= zero && v >= zero && u + v <= det;
		test = test /*&& idet > zero*/ && dist >= zero && dist < distance;

		ctx.distance[q] = Condition(test, dist, distance);
	}
}
void Triangle::Collide(SecondaryContext &ctx, int idx, int first, int last) const {
	Vec3q tnrm(plane.x, plane.y, plane.z);
	Vec3q ta(a), tca(ca), tba(ba);
	floatq zero(0.0f), one(1.0f), tit0(it0);

	int count = last - first + 1;
	for(int q = 0; q < count; q++) {
		int tq = q + first;

		const Vec3q dir = ctx.rayDir[tq];
		floatq idet = Inv(dir | tnrm);

		Vec3q tvec = ctx.rayOrigin[tq] - ta;
		floatq dist = -(tvec | tnrm) * idet;
		Vec3q tvec0 = tba ^ tvec;
		Vec3q tvec1 = tvec ^ tca;

		idet *= tit0;
		floatq v = (dir | tvec0) * idet;
		floatq u = (dir | tvec1) * idet;

		f32x4b test = Min(u, v) >= zero && u + v <= one;
		test = test && idet > zero && dist >= zero && dist < ctx.distance[tq];

		ctx.distance[tq] = Condition(test, dist, ctx.distance[tq]);
		ctx.normals[tq] = Condition(test, tnrm, ctx.normals[tq]);
		ctx.triIds[tq] = Condition(i32x4b(test), idx, ctx.triIds[tq]);
		ctx.barycentric[tq] = Condition(test, Vec2q(u, v), ctx.barycentric[tq]);
	}
}

bool Triangle::TestInterval(const RayInterval &i) const {
	//TODO: min/max origin
	float det =
		(plane.x < 0.0f? i.minDir.x : i.maxDir.x) * plane.x +
		(plane.y < 0.0f? i.minDir.y : i.maxDir.y) * plane.y +
		(plane.z < 0.0f? i.minDir.z : i.maxDir.z) * plane.z;
	if(det < 0.0f)
		return 0;

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

