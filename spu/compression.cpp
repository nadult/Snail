#include "spu/base.h"
#include "spu/trace.h"
#include <spu_intrinsics.h>

typedef vector unsigned int vint;

template <int size>
void Trans(unsigned char *data) {
	for(int x = size - 1; x > 0; x--)
		data[x] = data[x] - data[x - 1];
}

template <int size>
void TransV(unsigned char *data) {
	for(int y = size - 1; y > 0; y--)
		data[y * size] = data[y * size] - data[y * size - size];
}

void Compress(unsigned char *data) {
//	for(int y = 0; y < height; y++)
//		Trans<32>(data + y * width);
}
