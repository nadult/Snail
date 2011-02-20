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

	const Vec3f Color() const {
		return Vec3f(color[0], color[1], color[2]) * (1.0f / 255.0f);
	}

	Vec3f position;
	Vec3f direction;
	u8 color[4];
	u32 temp;
};

struct PhotonNode {
	PhotonNode() { }
	PhotonNode(unsigned count, unsigned first) :count(count), childAxis(3 << 30), first(first) { }
	PhotonNode(unsigned axis, float split, unsigned child) :childAxis(child | (axis << 30)), split(split) { }

	unsigned Left() const { return (childAxis & 0x3fffffff) + 0; }
	unsigned Right() const { return (childAxis & 0x3fffffff) + 1; }
	unsigned Axis() const { return childAxis >> 30; }
	float Split() const { return split; }

	bool IsLeaf() const { return Axis() == 3; }

	unsigned Count() const { return count; }
	unsigned First() const { return first; }

	unsigned count, childAxis, first;
	float split;
};

namespace shading { class Sample; }

int GatherPhotons(const vector<PhotonNode> &nodes, const vector<Photon> &photons,
					shading::Sample *samples, unsigned count, float range, const BBox &sceneBox);

void MakePhotonTree(vector<PhotonNode> &nodes, vector<Photon> &photons);
void TracePhotons(vector<Photon> &out, const Scene<BVH> &scene, unsigned count);

#endif
