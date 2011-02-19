#ifndef RTRACER_PHOTONS_H
#define RTRACER_PHOTONS_H

#include "rtbase.h"
#include "bvh/tree.h"
#include "scene.h"

struct Photon {
	Photon() { }
	Photon(const Vec3f &pos, const Vec3f &dir, const Vec3f &colorf)
		:position(pos), direction(dir) {
		color[0] = u8(Clamp(colorf.x * 255.0f, 0.0f, 255.0f));
		color[1] = u8(Clamp(colorf.y * 255.0f, 0.0f, 255.0f));
		color[2] = u8(Clamp(colorf.z * 255.0f, 0.0f, 255.0f));
	}
	Vec3f position;
	Vec3f direction;
	u8 color[4];
	u32 temp;
};


void TracePhotons(vector<Photon> &out, const Scene<BVH> &scene, unsigned count);

#endif
