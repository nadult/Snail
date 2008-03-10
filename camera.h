#ifndef CAMERA_H
#define CAMERA_H

#include "rtbase.h"


class Camera
{
public:
	Camera() :pos(0,0,0),right(1,0,0),up(0,1,0),front(0,0,1),plane_dist(0.5f)
		{ }

	Vec3f pos,right,front,up;
	float plane_dist;
};


#endif

