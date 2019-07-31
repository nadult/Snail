#include "rtbase.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <ostream>

void FileModTime(const string &path, time_t *out) {
	struct stat attrib;
	stat(path.c_str(), &attrib);
	*out = attrib.st_mtime;
}



int gVals[16] = { 0, };

std::ostream &operator<<(std::ostream &str, const floatq &v) {
	return str << "[" << v[0] << ", " << v[1] << ", " << v[2] << ", " << v[3] << "]";
}

std::ostream &operator<<(std::ostream &str, const Vec3f &v) {
	return str << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}

std::ostream &operator<<(std::ostream &str, const Plane &p) {
	return str << "(" << p.normal << " " << p.distance << ")";
}

bool IsNan(const Vec3f f) {
	floatq t(f.x, f.y, f.z, f.z);
	return ForAny(t != t);
}

bool IsNan(const Vec4f f) {
	floatq t(f.x, f.y, f.z, f.w);
	return ForAny(t != t);
}

bool IsNan(const Vec3q f) {
	return ForAny( f.x != f.x || f.y != f.y || f.z != f.z);
}

void Intersect(const Plane &a, const Plane &b, Vec3f &dir, Vec3f &point) {
	float n00 = a.normal | a.normal;
	float n01 = a.normal | b.normal;
	float n11 = b.normal | b.normal;
	float det = n00 * n11 - n01 * n01;

	if(det < constant::epsilon) {
	}

	float idet = 1.0f / det;
	float c0 = (n11 * a.distance - n01 * b.distance) * idet;
	float c1 = (n00 * b.distance - n01 * a.distance) * idet;

	dir = a.normal ^ b.normal;
	point = a.normal * c0 + b.normal * c1;
}

void ComputeMinMax(const Vec3q *vec, int size, Vec3f *outMin, Vec3f *outMax) {
	Vec3q min = vec[0], max = vec[0];
	for(int q = 1; q < size; q++) {
		min = VMin(min, vec[q]);
		max = VMax(max, vec[q]);
	}
	
	*outMin = Minimize(min);
	*outMax = Maximize(max);
}

#include "ray_group.h"
void ComputeMinMax(const Vec3q *vec, char *mask, int size, Vec3f *outMin, Vec3f *outMax) {
	RaySelector sel(mask, size);

	Vec3q min, max;
	int q = 0;
	while(!sel[q] && q < size) q++;
	if(q == size) {
		*outMin = *outMax = Vec3f(0.0f, 0.0f, 0.0f);
		return;
	}

	for(int k = 0; k < 4; k++) if(sel[q] & (1 << k)) {
		max = min = (Vec3q)ExtractN(vec[q], k);
		break;
	}

	for(;q < size; q++) {
		f32x4b mask = sel.SSEMask(q);
		min = Condition(mask, VMin(min, vec[q]), min);
		max = Condition(mask, VMax(max, vec[q]), max);
	}

	*outMin = Minimize(min);
	*outMax = Maximize(max);
}

void ComputeMinMax(const Vec3q *vec, const floatq *distMask, int size, Vec3f *outMin, Vec3f *outMax) {
	Vec3q min, max;
	int q = 0;
	while(!ForAny(distMask[q] >= 0.0f) && q < size) q++;
	if(q == size) {
		*outMin = *outMax = Vec3f(0.0f, 0.0f, 0.0f);
		return;
	}

	for(int k = 0; k < 4; k++) if(distMask[q][k] >= 0.0f) {
		max = min = (Vec3q)ExtractN(vec[q], k);
		break;
	}

	for(;q < size; q++) {
		f32x4b mask = distMask[q] >= 0.0f;
		min = Condition(mask, VMin(min, vec[q]), min);
		max = Condition(mask, VMax(max, vec[q]), max);
	}

	*outMin = Minimize(min);
	*outMax = Maximize(max);
}

