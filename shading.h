#pragma once

#include "rtbase.h"

namespace sampling { struct Cache; }

namespace shading {

	enum { blockSize = 4 };

	struct Sample {
		floatq distance;

		Vec3q position;
		Vec3q normal;
		Vec3q reflection;

		Vec2q texCoord;
		Vec2q texDiff;

		Vec3q diffuse;
		Vec3q specular;
		floatq opacity;

		Vec3q temp1;
		Vec3q temp2;
	};

}
