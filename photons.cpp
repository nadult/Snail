#include "photons.h"
#include <tr1/random>
#include <algorithm>
#include "shading.h"


struct OrderPhotons {
	OrderPhotons(int ax) :axis(ax) { }
	bool operator()(const Photon &a, const Photon &b) const
		{ return (&a.position.x)[axis] < (&b.position.x)[axis]; }

	int axis;
};

void MakePhotonTree(vector<PhotonNode> &nodes, Photon *photons, unsigned nodeIdx,
					unsigned first, unsigned count, const BBox &box, unsigned minNode, int level) {
/*	if(box.Width() < 0.25f && box.Height() < 0.25f && box.Width() < 0.25f) {
		Vec3f sumPos(0, 0, 0), sumCol(0, 0, 0);
		for(unsigned int n = 0; n < count; n++) {
			sumPos += photons[n].position;
			sumCol += photons[n].Color();
		}

		sumPos /= float(count);
		photons[0] = Photon(sumPos, Vec3f(0, 1, 0), sumCol);
		count = 1;
	} */

	int axis = MaxAxis(box.max - box.min);
	size_t mid = count / 2;

	std::nth_element(photons + first, photons + first + mid, photons + first + count, OrderPhotons(axis));
	float split = (&photons[first + mid].position.x)[axis];

	nodes[nodeIdx] = PhotonNode(axis, split, nodeIdx * 2 + 1);
	nodes[nodeIdx].count = count;
	nodes[nodeIdx].first = first;
	
	unsigned leftIdx = nodeIdx * 2 + 1;
	unsigned rightIdx = nodeIdx * 2 + 2;


	if(mid <= minNode) {
		nodes[leftIdx] = PhotonNode(mid, first);
		nodes[rightIdx] = PhotonNode(count - mid, first + mid);
		return;
	}

	Vec3f newMin = box.min, newMax = box.max;
	newMin[axis] = split; newMax[axis] = split;

	MakePhotonTree(nodes, photons, leftIdx, first, mid, {box.min, newMax}, minNode, level + 1);
	MakePhotonTree(nodes, photons, rightIdx, first + mid, count - mid, {newMin, box.max}, minNode, level + 1);
}

void MakePhotonTree(vector<PhotonNode> &nodes, vector<Photon> &photons) {
	nodes.resize(photons.size() * 4 + 16);
	BBox box(photons[0].position, photons[0].position);
	for(size_t n = 0; n < photons.size(); n++) {
		Vec3f point = photons[n].position;
		box.min = VMin(box.min, point);
		box.max = VMax(box.max, point);
	}

	MakePhotonTree(nodes, &photons[0], 0, 0, photons.size(), box, 4, 0);
}

int GatherPhotons(const vector<PhotonNode> &nodes, const vector<Photon> &photons,
					shading::Sample *samples, unsigned count, float range, const BBox &sceneBox)
{
	int stack[128];
	BBox boxStack[128];
	int stackPos = 0;
	int visitedNodes = 0;

	floatq rangeSq = range * range;
	floatq hrange = range * 0.5f;
	floatq mul = 0.2 * (1.0f / 255.0f) * (1.0f / range);
	
	boxStack[stackPos] = BBox(sceneBox.min - Vec3f(range, range, range), sceneBox.max + Vec3f(range, range, range));
	stack[stackPos++] = 0;

	for(int q = 0; q < count; q++)
		samples[q].temp1 = Vec3q(0.0f, 0.0f, 0.0f);

	int nPhotons = 0;

	while(stackPos) {
		int nodeIdx = stack[--stackPos];
		BBox bbox = boxStack[stackPos];
		const PhotonNode &node = nodes[nodeIdx];
		visitedNodes++;

		if(node.IsLeaf()) {
			int first = node.first;
			nPhotons += node.Count();

			Vec3q min(1.0f, 1.0f, 1.0f);

			for(int n = 0; n < node.Count(); n++) {
				const Photon &photon = photons[first + n];
				Vec3q posq(photon.position.x, photon.position.y, photon.position.z);
				Vec3q color = Vec3q(photon.color[0], photon.color[1], photon.color[2]) * mul;
				Vec3f normal = photon.direction;

				for(int q = 0; q < count; q++) {
					floatq dist = Length(posq - samples[q].position);
					floatq weight = Max(floatq(range) - dist, floatq(0.0f));
					weight *= Max(samples[q].normal | normal, floatq(0.0f));
					samples[q].temp1 += color * weight;
					min = VMin(min, samples[q].temp1);
				}
			}

		//	Vec3f tmin = Minimize(min);
		//	if(tmin.x >= 1.0f || tmin.y >= 1.0f || tmin.z >= 1.0f)
		//		goto END;
		}
		else {
			int axis = node.Axis();

			BBox leftBox = bbox, rightBox = bbox;
			(&leftBox.max.x)[axis] = node.Split() + range;
			(&rightBox.min.x)[axis] = node.Split() - range;

			Vec3q bmin(leftBox.min), bmax(leftBox.max);

			for(unsigned q = 0; q < count; q++) {
				const Vec3q pos = samples[q].position;

				if(ForAny(	pos.x >= bmin.x && pos.y >= bmin.y && pos.z >= bmin.z &&
							pos.x <= bmax.x && pos.y <= bmax.y && pos.z <= bmax.z)) {
					boxStack[stackPos] = leftBox;
					stack[stackPos++] = node.Left();
					break;
				}
			}

			bmin = Vec3q(rightBox.min), bmax = Vec3q(rightBox.max);
	
			for(unsigned q = 0; q < count; q++) {
				const Vec3q pos = samples[q].position;
				
				if(ForAny(	pos.x >= bmin.x && pos.y >= bmin.y && pos.z >= bmin.z &&
							pos.x <= bmax.x && pos.y <= bmax.y && pos.z <= bmax.z)) {
					boxStack[stackPos] = rightBox;
					stack[stackPos++] = node.Right();
					break;
				}
			}
		}
	}

	
END:	
//	for(int q = 0; q < count; q++)
//		samples[q].temp1 = VMin(samples[q].temp1, Vec3q(1.0f, 1.0f, 1.0f));
	return nPhotons;
}


//source:: PBRT
Vec3f UniformSampleHemisphere(float u1, float u2) {
	float z = u1;
	float r = Sqrt(Max(0.0f, 1.0f - z * z));
	float phi = 2 * constant::pi * u2;
	float x = r * Cos(phi);
	float y = r * Sin(phi);
	return Vec3f(x, y, z);
}

//source: PBRT
Vec3f UniformSampleSphere(float u1, float u2) {
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

			int tcount = sqrt(count);
			float itcount = 1.0f / float(tcount);

			int scount = Min(count - k * step, step);
			for(unsigned n = 0; n < scount; n++) {
				Vec3f pos = light.pos;
				float u1 = (float((n + k * step) % tcount) + 0.75f * (rand() - 0.5f)) * itcount;
				float u2 = (float((n + k * step) / tcount) + 0.75f * (rand() - 0.5f)) * itcount;

				//float u1 = rand(), u2 = rand();
				Vec3f dir = UniformSampleSphere(u1, u2);

				dirs[n] = dir; float lightSize = 4.0f;
				Vec3f offset(1.0f / 0.0f, 0, 0);
				while(LengthSq(offset) > lightSize)
					offset = Vec3f(rand() - 0.5f, rand() - 0.5f, rand() -0.5f) * lightSize;
				origins[n] = light.pos + offset;
				dists[n] = constant::inf;
				indices[n] = n;
				idirs[n] = VInv(dirs[n]);
			}

			scene.geometry.WideTrace(origins, dirs, idirs, indices, dists, scount);
	
			int offset = l * count + k * step;
			for(unsigned n = 0; n < scount; n++) {
				out[offset + n] = Photon(origins[n] + dirs[n] * dists[n], dirs[n], light.color);
				out[offset + n].temp = dists[n] == constant::inf?0 : 1;
			}
		}
	}

	out.resize(remove_if(out.begin(), out.end(), [](const Photon &p) { return !p.temp; }) - out.begin());
}
