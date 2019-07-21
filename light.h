#pragma once

#include "rtbase.h"

class Light
{
public:
	Light() { }
	Light(Vec3f p,Vec3f c,float rad) {
		Convert(p,pos); Convert(c,color);
		radius=rad; iRadius=1.0f/radius;
		radSq=rad*rad;
	}

	Vec3f pos,color;
	float radius,radSq,iRadius;
};
