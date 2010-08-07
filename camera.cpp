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
	std::map<string,Camera>::iterator it=data.begin();
	int count=data.size();
	sr&count;

	if(sr.IsLoading()) {
		for(int n=0;n<count;n++) {
			string str; Camera cam;
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

Camera::Camera()
	:pos(0, 0, 0), ang(0), pitch(0), plane_dist(1.0) { }
Camera::Camera(const Vec3f p, float ang, float pitch)
	:pos(pos), ang(ang), pitch(pitch), plane_dist(1.0) { }

void Camera::Print() {
	printf("Camera(Vec3f(%.4f, %.4f, %.4f),%.4f, %.4f);\n",
			pos.x, pos.y, pos.z, ang, pitch);
}
void Camera::Serialize(Serializer &sr) {
	sr & pos & ang & pitch & plane_dist;
}

void Camera::Move(const Vec3f shift) {
	pos += shift;
}

void Camera::SetPos(const Vec3f newPos) {
	pos = newPos;
}

void Camera::Rotate(float a) {
	ang += a;
}

void Camera::RotateY(float p) {
	pitch = Clamp(pitch + p, -constant::pi / 3.0f, constant::pi / 3.0f);
}

const Vec3f Camera::Pos() const {
	return pos;
}

void Camera::GetRotation(Vec3f &oRight, Vec3f &oUp, Vec3f &oFront) const {
	Vec3f x[2], y[2], z[2];

	::Rotate(Vec3f(0, 1, 0), ang, x[0], y[0], z[0]);
	::Rotate(Vec3f(1, 0, 0), pitch, x[1], y[1], z[1]);

	oRight = Vec3f(x[1] | x[0], x[1] | y[0], x[1] | z[0]);
	oUp    = Vec3f(y[1] | x[0], y[1] | y[0], y[1] | z[0]);
	oFront = Vec3f(z[1] | x[0], z[1] | y[0], z[1] | z[0]);
}

void CameraConfigs::AddConfig(const string &str,const Camera &cam) {
	data[str] = cam;
}

bool CameraConfigs::GetConfig(const string &str, Camera &cam) const {
	std::map<string,Camera>::const_iterator it = data.find(str);

	if(it != data.end()) {
		cam = it->second;
		return 1;
	}

	return 0;
}
