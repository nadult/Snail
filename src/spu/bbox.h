#pragma once

#include "spu/base.h"

class RayInterval;

struct BBox {
	Vec3f min, max;

	bool TestInterval(const RayInterval &i) const;
	template <class Context>
	bool Test(Context &ctx, int &firstActive, int &lastActive) const;
	bool Test(SecondaryContext &ctx, int &firstActive, int &lastActive) const;
};
