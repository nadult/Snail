#ifndef LIGHT_H
#define LIGHT_H

#include "rtbase.h"

class Light
{
public:
	Light() { }
	Light(Vec3f p,Vec3f c) {
		Convert(p,pos); Convert(c,color);
		zeroDist=Sqrt(Max(color.x,Max(color.y,color.z)))*256.0f;
	}

	Vec3p pos,color;
	float zeroDist;
};

#endif

