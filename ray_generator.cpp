#include "rtbase.h"
#include "ray_generator.h"

RayGenerator::RayGenerator(int level, int tw, int th, float pd, Vec3f right, Vec3f up, Vec3f front)
		:invW(1.0f/float(tw)), invH(1.0f/float(th)), planeDist(pd), level(level), w(tw), h(th) {
	invW *= float(w) / float(h);

	floatq taddx = floatq(0.0f, 1.0f, 0.0f, 1.0f) - floatq(w * 0.5f);
	floatq taddy = floatq(0.0f, 0.0f, 1.0f, 1.0f) - floatq(h * 0.5f);

	tright = right * invW;
	tup = up * invH;	
	txyz = Vec3q(tright) * taddx + Vec3q(tup) * taddy + Vec3q(front * planeDist);
}

void RayGenerator::Generate(int pw,int ph,int x,int y,Vec3q *out) const {
	Generate(level, pw, ph, x, y, out);
}

static const int tx[16] = { 0, 2, 0, 2, 4, 6, 4, 6, 0, 2, 0, 2, 4, 6, 4, 6 };
static const int ty[16] = { 0, 0, 2, 2, 0, 0, 2, 2, 4, 4, 6, 6, 4, 4, 6, 6 };

void RayGenerator::Generate(int level,int pw,int ph,int x,int y, Vec3q *out) const {
	{
		Assert(level == 3);

		GridSampler sampler;
		Vec3q right(tright), up(tup), tadd = txyz;
		floatq xoff(x + 0, x + 0, x + 2, x + 2);
		floatq yoff(y + 0, y + 0, y - 1, y - 1);

		for(int ty = 0; ty < 16; ty++) {
			floatq tposx[4] = { floatq(0.0f) + xoff, floatq(4.0f) + xoff,
								floatq(8.0f) + xoff, floatq(12.0f) + xoff };
			floatq tposy = floatq(ty) + yoff;

			Vec3q tpoint = up * tposy + tadd;
			Vec3q points[4] = { right * tposx[0] + tpoint, right * tposx[1] + tpoint,
								right * tposx[2] + tpoint, right * tposx[3] + tpoint };

			out[ty * 4 + 0] = points[0] * RSqrt(points[0] | points[0]);
			out[ty * 4 + 1] = points[1] * RSqrt(points[1] | points[1]);
			out[ty * 4 + 2] = points[2] * RSqrt(points[2] | points[2]);
			out[ty * 4 + 3] = points[3] * RSqrt(points[3] | points[3]);
		}
		return;
	}

	if(level > 2) {
		int npw = pw >> 1, nph = ph >> 1, nl = level - 1;

		Generate(nl, npw, nph, x      , y      , out + 0);
		Generate(nl, npw, nph, x + npw, y      , out + (1 << nl*2));
		Generate(nl, npw, nph, x      , y + nph, out + (2 << nl*2));
		Generate(nl, npw, nph, x + npw, y + nph, out + (3 << nl*2));
		return;
	}

	GridSampler sampler;
	Vec3q right(tright), up(tup), tadd = txyz;

	if(level == 2) {
		for(int t = 0; t < 16; t++) {
			Vec2f tpos = sampler(x + tx[t], y + ty[t]);
			Vec3q points = right * floatq(tpos.x) + up * floatq(tpos.y) + tadd;
			out[t] = points * RSqrt(points | points);
		}
	}
	else if(level == 1) {
		for(int t = 0; t < 4; t++) {
			Vec2f tpos = sampler(x + tx[t], y + ty[t]);
			Vec3q points = right * floatq(tpos.x) + up * floatq(tpos.y) + tadd;
			out[t] = points * RSqrt(points | points);
		}
	}
	else { // level == 0
		Vec2f tpos = sampler(x, y);
		Vec3q points = right * floatq(tpos.x) + up * floatq(tpos.y) + tadd;
		out[0] = points * RSqrt(points | points);
	}
}

void RayGenerator::Decompose(const Vec3q *in, Vec3q *out) const {
//	return;

	const int nQuads = 1 << (level * 2);

	if(level >= 1) {
		for(int n = 0; n < nQuads; n += 2) {
#if defined(VECLIB_SSE)
			__m128 tmp;
			tmp =
				_mm_shuffle_<0+(2<<2)+(1<<4)+(3<<6)>(_mm_unpacklo_ps(in[n+0].x.m,in[n+1].x.m));
			out[n+1].x.m =
				_mm_shuffle_<0+(2<<2)+(1<<4)+(3<<6)>(_mm_unpackhi_ps(in[n+0].x.m,in[n+1].x.m));
			out[n+0].x.m=tmp;

			tmp =
				_mm_shuffle_<0+(2<<2)+(1<<4)+(3<<6)>(_mm_unpacklo_ps(in[n+0].y.m,in[n+1].y.m));
			out[n+1].y.m =
				_mm_shuffle_<0+(2<<2)+(1<<4)+(3<<6)>(_mm_unpackhi_ps(in[n+0].y.m,in[n+1].y.m));
			out[n+0].y.m=tmp;

			tmp =
				_mm_shuffle_<0+(2<<2)+(1<<4)+(3<<6)>(_mm_unpacklo_ps(in[n+0].z.m,in[n+1].z.m));
			out[n+1].z.m =
				_mm_shuffle_<0+(2<<2)+(1<<4)+(3<<6)>(_mm_unpackhi_ps(in[n+0].z.m,in[n+1].z.m));
			out[n+0].z.m=tmp;
#else
			float x[8], y[8], z[8];
			Convert(in[n + 0].x, x);
			Convert(in[n + 1].x, x + 4);
			out[n + 0].x = f32x4(x[0], x[1], x[4], x[5]);
			out[n + 1].x = f32x4(x[2], x[3], x[6], x[7]);
			
			Convert(in[n + 0].y, y);
			Convert(in[n + 1].y, y + 4);
			out[n + 0].y = f32x4(y[0], y[1], y[4], y[5]);
			out[n + 1].y = f32x4(y[2], y[3], y[6], y[7]);
			
			Convert(in[n + 0].z, z);
			Convert(in[n + 1].z, z + 4);
			out[n + 0].z = f32x4(z[0], z[1], z[4], z[5]);
			out[n + 1].z = f32x4(z[2], z[3], z[6], z[7]);
#endif
		}
	}
	if(level >= 2) {
		for(int n=0;n<nQuads;n+=8) {
			Vec3q *p=out+n,tmp;

			{ const Vec3q tmp=p[2]; p[2]=p[1]; p[1]=p[4]; p[4]=tmp; }
			{ const Vec3q tmp=p[3]; p[3]=p[5]; p[5]=p[6]; p[6]=tmp; }
		}
	}
	if(level >= 3) {
		for(int n=0;n<nQuads;n+=32) {
			Vec3q *p=out+n;

			for(int k=0;k<2;k++) {
				Vec3q *pk=p+k;
				{ const Vec3q tmp = pk[2]; pk[2]=pk[16]; pk[16]=pk[8]; pk[8]=pk[4]; pk[4]=tmp;  }
				{ const Vec3q tmp = pk[6]; pk[6]=pk[18]; pk[18]=pk[24]; pk[24]=pk[12]; pk[12]=tmp; }
				{ const Vec3q tmp = pk[14]; pk[14]=pk[22]; pk[22]=pk[26]; pk[26]=pk[28]; pk[28]=tmp; }
				{ const Vec3q tmp = pk[10]; pk[10]=pk[20]; pk[20]=tmp; }
			}
		}
	}
}

