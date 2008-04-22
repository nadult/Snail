#ifndef LOADER_H
#define LOADER_H

#include "rtbase.h"
#include "object.h"

void LoadWavefrontObj(const char *fileName,vector<Triangle> &out,float scale);
void LoadRaw(const char *fileName,vector<Triangle> &out,float scale);

#endif

