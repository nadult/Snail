#ifndef RTRACER_CAMERA_H
#define RTRACER_CAMERA_H

#include "rtbase.h"
#include <stdio.h>
#include <map>

struct Camera {
	Camera() { }
	Camera(const Vec3f &pos, const Vec3f &right, const Vec3f &up, const Vec3f &front, float plane_dist)
		:plane_dist(plane_dist), pos(pos), right(right), up(up), front(front) { }

	float plane_dist;
	Vec3f pos, right, up, front;
};

class OrbitingCamera {
public:
	OrbitingCamera();
	operator const Camera() const;

	void Rotate(float);
	void RotateY(float);
	void Zoom(float);
	void Reset(Vec3f pos, float dist);

	double plane_dist;
	Vec3f target, pos, right;
};

class FPSCamera
{
public:
	FPSCamera(const Vec3f pos, float angle, float pitch);
	FPSCamera();

	operator const Camera() const;

	void Print();
	void serialize(Serializer&);

	void Rotate(float a);
	void RotateY(float p);
	void Move(const Vec3f);
	void SetPos(const Vec3f);

	const Vec3f Pos() const;

	float plane_dist;
//private:
	float ang, pitch;
	Vec3f pos;
};

class CameraConfigs {
public:
	void AddConfig(const string &fileName, const FPSCamera&);
	bool GetConfig(const string &fileName, FPSCamera&) const;
	void serialize(Serializer&);

	std::map<string, FPSCamera> data;
};


#endif

