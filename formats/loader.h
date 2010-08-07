#ifndef RTRACER_LOADER_H
#define RTRACER_LOADER_H

#include "rtbase.h"
#include "triangle.h"

struct ShadingData {
	ShadingData() { }
	ShadingData(const Vec3f &a,const Vec3f &b,const Vec3f &c) {
		nrm[0]=a; nrm[1]=b; nrm[2]=c;
		uv[0]=uv[1]=uv[2]=Vec2f(0,0);
	}
	Vec3f nrm[3]; // for every vertex
	Vec2f uv[3];
};

typedef vector<ShadingData,AlignedAllocator<ShadingData> > ShadingDataVec;

void LoadWavefrontObj(const char *fileName,TriVector &out,ShadingDataVec &shadingData,float scale,uint maxTris);
void LoadRaw(const char *fileName,TriVector &out,float scale,uint maxTris);
void LoadV3O(string fileName,TriVector &out,ShadingDataVec &shadingData,float scale,uint maxTris);
void LoadModel(const string &fileName,TriVector &out,ShadingDataVec &shadingData,float scale,uint maxTris);
void LoadProc(const string &fileName,TriVector &out,ShadingDataVec &shData,float scale,uint maxTris);

#endif

