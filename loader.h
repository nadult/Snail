#ifndef RTRACER_LOADER_H
#define RTRACER_LOADER_H

#include "rtbase.h"
#include "object.h"

void LoadWavefrontObj(const char *fileName,vector<Triangle> &out,float scale);
void LoadRaw(const char *fileName,vector<Triangle> &out,float scale);

#endif

