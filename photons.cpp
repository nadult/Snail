#include "photons.h"
#include <tr1/random>


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

void TracePhotons(vector<Photon> &out, const Scene<BVH> &scene, unsigned count) {
	InputAssert(count <= 65536);
	out.resize(scene.lights.size() * count);

#pragma omp parallel for
	for(unsigned l = 0; l < scene.lights.size(); l++) {
		const Light &light = scene.lights[l];
	
		std::tr1::variate_generator<std::tr1::mt19937, std::tr1::uniform_real<> >
			rand(std::tr1::mt19937(l), std::tr1::uniform_real<>(0.0, 1.0));

		Vec3f dirs[count], origins[count];
		float dists[count];
		u16 indices[count];

		for(unsigned n = 0; n < count; n++) {
			Vec3f pos = light.pos;
			float u1 = rand(), u2 = rand();
			Vec3f dir = UniformSampleSphere(u1, u2);

			dirs[n] = dir;
			origins[n] = light.pos;
			dists[n] = constant::inf;
			indices[n] = n;
		}

		scene.geometry.WideTrace(origins, dirs, 0, indices, dists, count);

		int offset = l * count;
		for(unsigned n = 0; n < count; n++) {
			out[offset + n] = Photon(
					origins[n] + dirs[n] * dists[n],
					-dirs[n],
					light.color * Clamp((50.0f - dists[n]) * 0.02f, 0.0f, 1.0f));
		}
	}
}
