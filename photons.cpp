#include "photons.h"
#include <tr1/random>
#include <algorithm>

//source:: PBRT
static Vec3f UniformSampleHemisphere(float u1, float u2) {
	float z = u1;
	float r = Sqrt(Max(0.0f, 1.0f - z * z));
	float phi = 2 * constant::pi * u2;
	float x = r * Cos(phi);
	float y = r * Sin(phi);
	return Vec3f(x, y, z);
}

//source: PBRT
static Vec3f UniformSampleSphere(float u1, float u2) {
	float z = 1.0f - 2.0f * u1;
	float r = Sqrt(Max(0.0f, 1.0f - z * z));
	float phi = 2 * constant::pi * u2;
	float x = r * Cos(phi);
	float y = r * Sin(phi);
	return Vec3f(x, y, z);
}

struct RayOrder {
	RayOrder(const Vec3f *dirs) :dirs(dirs) { }

	bool operator()(u16 idxA, u16 idxB) const {
		const Vec3f &a = dirs[idxA], &b = dirs[idxB];
		bool signa[3] = { a.x < 0.0f, a.y < 0.0f, a.z < 0.0f };
		bool signb[3] = { b.x < 0.0f, b.y < 0.0f, b.z < 0.0f };

		return signa[0] == signb[0]? signa[1] == signb[1]?
				signa[2] < signb[2] : signa[1] < signb[1] : signa[0] < signb[0];
	}

	const Vec3f *dirs;
};

void TracePhotons(vector<Photon> &out, const Scene<BVH> &scene, unsigned count) {
	out.resize(scene.lights.size() * count);

//#pragma omp parallel for
	for(unsigned l = 0; l < scene.lights.size(); l++) {
		const Light &light = scene.lights[l];
	
		enum { step = 8 * 1024 };
		int nsteps = (count + step) / step;

#pragma omp parallel for
		for(int k = 0; k < nsteps; k++) {
			std::tr1::variate_generator<std::tr1::mt19937, std::tr1::uniform_real<> >
				rand(std::tr1::mt19937((k + 1) * (l + 1)), std::tr1::uniform_real<>(0.0, 1.0));

			Vec3f dirs[step], origins[step];
			Vec3f idirs[step];
			float dists[step];
			u16 indices[step];

			int scount = Min(count - k * step, step);
			for(unsigned n = 0; n < scount; n++) {
				Vec3f pos = light.pos;
				float u1 = rand(), u2 = rand();
				Vec3f dir = UniformSampleSphere(u1, u2);

				dirs[n] = dir;
				origins[n] = light.pos + (Vec3f(rand(), rand(), rand()) - Vec3f(0.5f, 0.5f, 0.5f)) * 2.0f;
				dists[n] = constant::inf;
				indices[n] = n;
				idirs[n] = VInv(dirs[n]);
			}

			scene.geometry.WideTrace(origins, dirs, idirs, indices, dists, scount);
	
			int offset = l * count + k * step;
			for(unsigned n = 0; n < scount; n++) {
				out[offset + n] = Photon(
						origins[n] + dirs[n] * dists[n],
						-dirs[n],
						light.color * Clamp((50.0f - dists[n]) * 0.02f, 0.0f, 1.0f));
				out[offset + n].temp = dists[n] == constant::inf?0 : 1;
			}
		}
	}

	out.resize(remove_if(out.begin(), out.end(), [](const Photon &p) { return !p.temp; }) - out.begin());
}
