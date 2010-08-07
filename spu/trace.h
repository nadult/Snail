#ifndef RTRACER_SPU_SHARED_H
#define RTRACER_SPU_SHARED_H

#define ALIGN256 __attribute__((aligned(256)))

#include <veclib.h>

using namespace veclib;

struct TaskInfo {
	unsigned long long bvhNodes, bvhTris;
	unsigned int nbvhNodes, nbvhTris;

	unsigned long long pixelData;
	unsigned long long lightData;

	int startX, startY, width, height;
	int outWidth, outHeight;
	int nParts;

	int nLights;

	Vec3<float> camRight, camUp, camFront;
	Vec3<float> camPos;
	float camPlaneDist;

	char fill[128];
};

struct LightData {
	Vec3f pos, color;
	float radius, iRadius;
};

#endif
