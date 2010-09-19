#ifndef RTRACER_SPU_BBOX_H
#define RTRACER_SPU_BBOX_H

#include "spu/base.h"

class RayInterval;

struct BBox {
	Vec3f min, max;

	bool TestInterval(const RayInterval &i) const;
	template <class Context>
	bool Test(Context &ctx, int &firstActive, int &lastActive) const;
	bool Test(SecondaryContext &ctx, int &firstActive, int &lastActive) const;
};



#endif
