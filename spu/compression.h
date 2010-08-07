#ifndef RTRACER_SPU_COMPRESSION_H
#define RTRACER_SPU_COMPRESSION_H


#ifndef ALIGN256
#define ALIGN256 __attribute__((aligned(256)))
#endif

struct TaskInfo {
	unsigned long long data;
	int w, h, bpp, outSize;
	char fill[128];
};

#endif
