#ifndef RTRACER_SPU_BASE_H
#define RTRACER_SPU_BASE_H

#include <spu_mfcio.h>
#include "rtbase_math.h"

void Local2Mem(unsigned long long memAddr, volatile void *lsAddr, unsigned int size);
void Mem2Local(unsigned long long memAddr, volatile void *lsAddr, unsigned int size);

#endif

