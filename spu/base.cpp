#include "spu/base.h"

void Local2Mem(unsigned long long memAddr, volatile void *lsAddr, unsigned int size) {
	unsigned int tag = 0;
	unsigned int mask = 1;
	mfc_put(lsAddr, memAddr, size, tag, 0, 0);
	mfc_write_tag_mask(mask);
	mfc_read_tag_status_all();
}

void Mem2Local(unsigned long long memAddr, volatile void *lsAddr, unsigned int size) {
	unsigned int tag = 0;
	unsigned int mask = 1;
	mfc_get(lsAddr, memAddr, size, tag, 0, 0);
	mfc_write_tag_mask(mask);
	mfc_read_tag_status_all();
}

