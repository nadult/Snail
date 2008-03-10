#ifndef LOADER_H
#define LOADER_H

#include "rtbase.h"
#include "object.h"

void LoadWavefrontObj(const char *fileName,vector<Object> &out,float scale);
void LoadRaw(const char *fileName,vector<Object> &out,float scale);

#endif

