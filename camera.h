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
	operator Camera() const;

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

	operator Camera() const;

	void Print();
	void save(Stream&) const;
	void load(Stream&);

	void Rotate(float a);
	void RotateY(float p);
	void Move(const Vec3f);
	void SetPos(const Vec3f);

	const Vec3f Pos() const;

	float plane_dist = 1.0f;
//private:
	float ang = 0.0f, pitch = 0.0f;
	Vec3f pos = Vec3f(0.0f, 0.0f, 0.0f);
};

class CameraConfigs {
public:
	void AddConfig(const string &fileName, const FPSCamera&);
	bool GetConfig(const string &fileName, FPSCamera&) const;

	void save(Stream&) const;
	void load(Stream&);

	std::map<string, FPSCamera> data;
};


#endif

