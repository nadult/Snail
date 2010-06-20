#include "triangle.h"

bool Triangle::TestInterval(Vec3f orig, Vec3f minDir, Vec3f maxDir) const {
	Vec3f nrm = Nrm();
	float det =
		(nrm.x < 0.0f? minDir.x : maxDir.x) * nrm.x +
		(nrm.y < 0.0f? minDir.y : maxDir.y) * nrm.y +
		(nrm.z < 0.0f? minDir.z : maxDir.z) * nrm.z;

	Vec3f tvec = orig - a;
	Vec3f c1 = ba ^ tvec, c2 = tvec ^ ca;

	float u[2] = {
		(c1.x < 0.0f? maxDir.x : minDir.x) * c1.x +
		(c1.y < 0.0f? maxDir.y : minDir.y) * c1.y +
		(c1.z < 0.0f? maxDir.z : minDir.z) * c1.z,
		(c1.x < 0.0f? minDir.x : maxDir.x) * c1.x +
		(c1.y < 0.0f? minDir.y : maxDir.y) * c1.y +
		(c1.z < 0.0f? minDir.z : maxDir.z) * c1.z };
	float v[2] = {
		(c2.x < 0.0f? maxDir.x : minDir.x) * c2.x +
		(c2.y < 0.0f? maxDir.y : minDir.y) * c2.y +
		(c2.z < 0.0f? maxDir.z : minDir.z) * c2.z,
		(c2.x < 0.0f? minDir.x : maxDir.x) * c2.x +
		(c2.y < 0.0f? minDir.y : maxDir.y) * c2.y +
		(c2.z < 0.0f? minDir.z : maxDir.z) * c2.z };

	return Min(u[1], v[1]) >= 0.0f && u[0] + v[0] <= det * ca.t0;
}
