#ifndef RTRACER_SPU_TRACE_H
#define RTRACER_SPU_TRACE_H

#define ALIGN256 __attribute__((aligned(256)))

typedef unsigned long long cond_ea_t;
typedef unsigned long long mutex_ea_t;

enum {
	maxThreads = 16,
	blockWidth = 16,
	blockHeight = 64,
	QuadLevels = 3,
	NQuads = 1 << (QuadLevels * 2),
	PWidth = 2 << QuadLevels,
	PHeight = 2 << QuadLevels,
};

struct TasksInfo {
	mutex_ea_t mutex;
	cond_ea_t finishCond, newTaskCond; 
	unsigned long long taskList;
	unsigned int nTasks, nTaken, nAllTasks, nFinished;
	bool pleaseDie[maxThreads];
	short offsets[maxThreads]; // current offset into pixel data buffer
	
	int nRaysTraced;
	int nTrisIntersected;
	int nBVHIterations;
	int temp;

	int timers[8];
};

struct TaskInfo {
	unsigned long long bvhNodes, bvhTris;
	unsigned long long bvhShTris;

	unsigned long long rData, gData, bData;
	unsigned long long lightData;
	unsigned long long texInfoData;
	
	unsigned int nbvhNodes, nbvhTris;

	int startX, startY, width, height;
	int outWidth, outHeight;
	int nLights;

	int nodeId;

	Vec3<float> camRight, camUp, camFront;
	Vec3<float> camPos;
	float camPlaneDist;
	
	short dataOffset;
	bool colorizeNodes;
	bool antialias;
	bool reflections;
	bool shading;
	bool statsVis;
	bool compress;

	char fill[92];
};


struct LightData {
	Vec3f pos, color;
	float radius, iRadius;
};

struct TextureInfo {
	unsigned long long dataPtr;
	unsigned short width, height;
	unsigned short wMask, hMask;
};

#endif
