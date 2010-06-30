#ifndef RTRACER_CAMERA_H
#define RTRACER_CAMERA_H

#include "rtbase.h"
#include <stdio.h>
#include <map>


class Camera
{
public:
	Camera(const Vec3f pos, float angle, float pitch);
	Camera();

	void Print();
	void Serialize(Serializer&);

	void Rotate(float a);
	void RotateY(float p);
	void Move(const Vec3f);
	void SetPos(const Vec3f);

	const Vec3f Pos() const;
	void GetRotation(Vec3f &right, Vec3f &up, Vec3f &front) const;

	float plane_dist;
private:
	float ang, pitch;
	Vec3f pos;
};

class CameraConfigs {
public:
	void AddConfig(const string &fileName, const Camera&);
	bool GetConfig(const string &fileName, Camera&) const;
	void Serialize(Serializer&);

	std::map<string, Camera> data;
};


#endif

