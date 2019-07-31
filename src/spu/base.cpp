#include "spu/base.h"

void Local2Mem(unsigned long long memAddr, volatile void *lsAddr, unsigned int size, int tag) {
	mfc_put(lsAddr, memAddr, size, tag, 0, 0);
	mfc_write_tag_mask(1 << tag);
	mfc_read_tag_status_all();
}

void Mem2Local(unsigned long long memAddr, volatile void *lsAddr, unsigned int size, int tag) {
	mfc_get(lsAddr, memAddr, size, tag, 0, 0);
	mfc_write_tag_mask(1 << tag);
	mfc_read_tag_status_all();
}

