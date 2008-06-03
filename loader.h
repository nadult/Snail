#ifndef RTRACER_LOADER_H
#define RTRACER_LOADER_H

#include "rtbase.h"
#include "object.h"

struct ShadingData {
	Vec3p nrm[3]; // for every vertex
};

typedef vector<ShadingData,AlignedAllocator<ShadingData> > ShadingDataVec;

void LoadWavefrontObj(const char *fileName,TriVector &out,ShadingDataVec &shadingData,float scale,uint maxTris);
void LoadRaw(const char *fileName,TriVector &out,float scale,uint maxTris);

#endif

