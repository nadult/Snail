#ifndef RTRACER_CAMERA_H
#define RTRACER_CAMERA_H

#include "rtbase.h"
#include <map>
#include <stdio.h>


class Camera
{
public:
	Camera() :pos(0,0,0),right(1,0,0),up(0,1,0),front(0,0,1),plane_dist(0.5f) { }
	Camera(Vec3f p,Vec3f f,Vec3f r,Vec3f u=Vec3f(0,1,0)) :pos(p),right(r),front(f),up(u),plane_dist(0.5f) { }

	void Print() {
		printf("Camera(Vec3f(%.4f,%.4f,%.4f),Vec3f(%.4f,%.4f,%.4f),Vec3f(%.4f,%.4f,%.4f));\n",
				pos.x,pos.y,pos.z,front.x,front.y,front.z,right.x,right.y,right.z);
	}
	void Serialize(Serializer &sr) {
		sr & pos & right & front & up &plane_dist;
	}

	Vec3f pos,right,front,up;
	float plane_dist;
};

class CameraConfigs {
public:
	void AddConfig(const string &fileName,const Camera&);
	bool GetConfig(const string &fileName,Camera&) const;
	void Serialize(Serializer&);

	std::map<string,Camera> data;
};


#endif

