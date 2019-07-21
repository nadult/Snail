#pragma once


#include "spu/base.h"

void InitTexCache(unsigned long long texInfoPtr);

const Vec3q SampleTexture(int texId, const Vec2q uv);
