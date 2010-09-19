#include "camera.h"

static void Rotate(const Vec3f axis, float radians, Vec3f &v1, Vec3f &v2, Vec3f &v3) {
    float cos = Cos(radians), sin = Sin(radians);
    float oneMinusCos = 1.0f - cos;

    float xx = axis[0] * axis[0];
    float yy = axis[1] * axis[1];
    float zz = axis[2] * axis[2];
    float xym = axis[0] * axis[1] * oneMinusCos;
    float xzm = axis[0] * axis[2] * oneMinusCos;
    float yzm = axis[1] * axis[2] * oneMinusCos;
    float xSin = axis[0] * sin;
    float ySin = axis[1] * sin;
    float zSin = axis[2] * sin;
    
	v1 = Vec3f(xx * oneMinusCos + cos, xym + zSin, xzm - ySin);
	v2 = Vec3f(xym - zSin, yy * oneMinusCos + cos, yzm + xSin);
	v3 = Vec3f(xzm + ySin, yzm - xSin, zz * oneMinusCos + cos);
}

void CameraConfigs::Serialize(Serializer &sr) {
	std::map<string, FPSCamera>::iterator it=data.begin();
	int count=data.size();
	sr&count;

	if(sr.IsLoading()) {
		for(int n=0;n<count;n++) {
			string str; FPSCamera cam;
			sr & str & cam;
			data[str] = cam;
		}
	}
	else {
		while(it!=data.end()) {
			string tmp=it->first;
			sr & tmp & it->second;
			++it;
		}
	}
}

FPSCamera::FPSCamera()
	:pos(0, 0, 0), ang(0), pitch(0), plane_dist(1.0) { }
FPSCamera::FPSCamera(const Vec3f p, float ang, float pitch)
	:pos(pos), ang(ang), pitch(pitch), plane_dist(1.0) { }



void FPSCamera::Print() {
	printf("FPSCamera(Vec3f(%.4f, %.4f, %.4f), %.4f, %.4f);\n",
			pos.x, pos.y, pos.z, ang, pitch);
}

FPSCamera::operator const Camera() const {
	Vec3f x[2], y[2], z[2];

	::Rotate(Vec3f(0, 1, 0), ang, x[0], y[0], z[0]);
	::Rotate(Vec3f(1, 0, 0), pitch, x[1], y[1], z[1]);

	Vec3f right = Vec3f(x[1] | x[0], x[1] | y[0], x[1] | z[0]);
	Vec3f up    = Vec3f(y[1] | x[0], y[1] | y[0], y[1] | z[0]);
	Vec3f front = Vec3f(z[1] | x[0], z[1] | y[0], z[1] | z[0]);

	return Camera(pos, right, up, front, plane_dist);
}

void FPSCamera::Serialize(Serializer &sr) {
	sr & pos & ang & pitch & plane_dist;
}

void FPSCamera::Move(const Vec3f shift) {
	pos += shift;
}

void FPSCamera::SetPos(const Vec3f newPos) {
	pos = newPos;
}

void FPSCamera::Rotate(float a) {
	ang += a;
}

void FPSCamera::RotateY(float p) {
	pitch = Clamp(pitch + p, -constant::pi / 3.0f, constant::pi / 3.0f);
}

const Vec3f FPSCamera::Pos() const {
	return pos;
}

OrbitingCamera::OrbitingCamera()
	:pos(0, 0, 0), ang(0), pitch(0), plane_dist(1.0) { }
	
OrbitingCamera::operator const Camera() const {
	Vec3f x[2], y[2], z[2];

	::Rotate(Vec3f(0, 1, 0), ang, x[0], y[0], z[0]);
	::Rotate(Vec3f(1, 0, 0), pitch, x[1], y[1], z[1]);

	Vec3f right = Vec3f(x[1] | x[0], x[1] | y[0], x[1] | z[0]);
	Vec3f up    = Vec3f(y[1] | x[0], y[1] | y[0], y[1] | z[0]);
	Vec3f front = Vec3f(z[1] | x[0], z[1] | y[0], z[1] | z[0]);

	return Camera(pos - front * dist, right, up, front, plane_dist);
}


void OrbitingCamera::Rotate(float a) {
	ang += a;
}

void OrbitingCamera::RotateY(float p) {
	pitch = Clamp(pitch + p, -constant::pi / 2.0f, constant::pi / 2.0f);
}

void OrbitingCamera::Zoom(float z) {
	dist += z;
	if(dist < 0) dist = 0;
}

void OrbitingCamera::SetPos(const Vec3f &newPos) {
	pos = newPos;
}


void CameraConfigs::AddConfig(const string &str,const FPSCamera &cam) {
	data[str] = cam;
}

bool CameraConfigs::GetConfig(const string &str, FPSCamera &cam) const {
	std::map<string, FPSCamera>::const_iterator it = data.find(str);

	if(it != data.end()) {
		cam = it->second;
		return 1;
	}

	return 0;
}
