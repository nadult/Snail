#ifndef LIGHT_H
#define LIGHT_H

#include "rtbase.h"

class Light
{
public:
	Light() { }
	Light(Vec3f p,Vec3f c) {
		Convert(p,pos); Convert(c,color);
		zeroDist=Sqrt(Max(color.X(),Max(color.Y(),color.Z())))*256.0f;
	}

	Vec3p pos,color;
	float zeroDist;
};

#endif

