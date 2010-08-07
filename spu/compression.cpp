#include "quicklz/quicklz.c"
#include "quicklz/quicklz.h"
#include "spu/compression.h"
#include "spu/base.h"

#include "spu/base.cpp"

TaskInfo info;

void Mem2LocalU(unsigned long long memAddr, volatile void *lsAddr, unsigned int size) {
	for(int b = 1024 * 16; b >= 1 && size; b >>= 1) {
		if(size >= b) {
			Mem2Local(memAddr, lsAddr, b);
			memAddr += b; lsAddr = (char*)lsAddr + b; size -= b;
		}
	}
}
void Local2MemU(unsigned long long memAddr, volatile void *lsAddr, unsigned int size) {
	for(int b = 1024 * 16; b >= 1 && size; b >>= 1) {
		if(size >= b) {
			Local2Mem(memAddr, lsAddr, b);
			memAddr += b; lsAddr = (char*)lsAddr + b; size -= b;
		}
	}
}

unsigned char buf[1024 * 16 * 3] ALIGN256;
unsigned char outBuf[1024 * 16 * 3 + 400] ALIGN256;
char scratch[QLZ_SCRATCH_COMPRESS] ALIGN256;

int main(unsigned long long speid, unsigned long long argp, unsigned long long envp) {
	Mem2Local(argp, &info, 128);

	int nPixels = info.w * info.h;
	unsigned char *channel[3] = { buf + nPixels * 0, buf + nPixels * 1, buf + nPixels * 2 };

	for(int n = 0; n < nPixels; n += 256) {
		int pixels = nPixels - n;
		if(pixels > 256) pixels = 256;
		char tbuf[768];
		if(pixels == 256)
			Mem2Local(info.data + n * 3, tbuf, 768);
		else
			Mem2LocalU(info.data + n * 3, tbuf, pixels * 3);

		for(int k = 0; k < pixels; k++) {
			channel[0][n + k] = tbuf[k * 3 + 0];
			channel[1][n + k] = tbuf[k * 3 + 1] - tbuf[k * 3 + 0];
			channel[2][n + k] = tbuf[k * 3 + 2] - tbuf[k * 3 + 0];
		}
	}

	info.outSize = qlz_compress((char*)buf, (char*)outBuf, nPixels * 3, scratch);
	for(int n = 0; n < info.outSize; n+=256) {
		int bytes = info.outSize - n;
		if(bytes > 256) bytes = 256;
		
		if(bytes == 256)
			Local2Mem(info.data + n, outBuf + n, 256);
		else
			Local2MemU(info.data + n, outBuf + n, bytes);
	}

	Local2Mem(argp, &info, 128);
}
