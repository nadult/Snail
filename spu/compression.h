#pragma once


#ifndef ALIGN256
#define ALIGN256 __attribute__((aligned(256)))
#endif

struct TaskInfo {
	unsigned long long data;
	int w, h, bpp, outSize;
	char fill[128];
};
