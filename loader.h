#ifndef RTRACER_LOADER_H
#define RTRACER_LOADER_H

#include "rtbase.h"
#include "object.h"

uint LoadWavefrontObj(const char *fileName,Vector<Triangle> &out,float scale,uint maxTris);
uint LoadRaw(const char *fileName,Vector<Triangle> &out,float scale,uint maxTris);

#endif

