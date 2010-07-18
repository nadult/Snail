#include "pch.h"
#include "quat.h"

Quat::Quat(const Matrix<Vec4f> &mat) {
	w = sqrt( Max( 0.0f, 1.0f + mat.x.x + mat.y.y + mat.z.z ) ) / 2.0f;
	x = sqrt( Max( 0.0f, 1.0f + mat.x.x - mat.y.y - mat.z.z ) ) / 2.0f;
	y = sqrt( Max( 0.0f, 1.0f - mat.x.x + mat.y.y - mat.z.z ) ) / 2.0f;
	z = sqrt( Max( 0.0f, 1.0f - mat.x.x - mat.y.y + mat.z.z ) ) / 2.0f;

	if(mat.z.y > mat.y.z) x = -x;
	if(mat.x.z > mat.z.x) y = -y;
	if(mat.y.x > mat.x.y) z = -z;
}

/*
Quat::operator Matrix3() const {
	float tx = x * 1.414213562373f;
	float ty = y * 1.414213562373f;
	float tz = z * 1.414213562373f;
	float tw = w * 1.414213562373f;

	float xx = tx * tx, yy = ty * ty, zz = tz * tz;
	float xz = tx * tz, xy = tx * ty, yz = ty * tz;
	float xw = tx * tw, yw = ty * tw, zw = tz * tw;

	return Matrix3(
			{ 1.0f - (yy + zz),	(xy + zw),			(xz - yw) },
			{ (xy - zw),		1.0f - (xx + zz),	(yz + xw) },
			{ (xz + yw),		(yz - xw),			1.0f - (xx + yy) });
}*/

Quat::Quat(float y, float p, float r) {
	float cy = Cos(y * 0.5f);
	float cp = Cos(p * 0.5f);
	float cr = Cos(r * 0.5f);

	float sy = Sin(y * 0.5f);
	float sp = Sin(p * 0.5f);
	float sr = Sin(r * 0.5f);

	x = cy * cp * sr - sy * sp * cr;
	y = cy * sp * cr + sy * cp * sr;
	z = sy * cp * cr - cy * sp * sr;
	w = cy * cp * cr + sy * sp * sr;
}

Quat::Quat(const AxisAngle &aa) {
	float half = 0.5f * aa.Angle();
	float tSin = Sin(half);
	*this = Normalize( Quat(Vec4f(tSin * aa.Axis().x, tSin * aa.Axis().y, tSin * aa.Axis().z, Cos(half))));
}

Quat::operator const AxisAngle() const {
	float sqrLen = Sqrt(x * x + y * y + z * z);
	return sqrLen > 0?
		AxisAngle(Vec3f(x, y, z) / sqrLen, 2.0f * acos(w)):
		AxisAngle(Vec3f(1.0f,0.0f,0.0f), 0.0f);
}

const Quat &Quat::operator*=(const Quat &q) {
	float tx = x, ty = y, tz = z;
	x = w * q.x + tx * q.w + ty * q.z - q.y * tz;
	y = w * q.y + ty * q.w + tz * q.x - q.z * tx;
	z = w * q.z + tz * q.w + tx * q.y - q.x * ty;
	w = w * q.w - tx * q.x - ty * q.y - q.z * tz;

	return *this;
}

const Quat &Quat::operator*=(const Vec3f &v) {
	return operator *= (Quat(Vec4f(v.x, v.y, v.z, 0.0f)));
}

const Quat operator*(const Vec3f& v, const Quat& q) {
	return Quat(Vec4f(v.x, v.y, v.z, 0.0f)) * q;
}

const Quat Inverse(const Quat &q) {
	return Quat(Vec4f(-q.x, -q.y, -q.z, q.w));
}

const Quat Lerp(const Quat &lhs, const Quat &rhs, float fT)
{
	//TODO testme
	float fCos = lhs | rhs;
	float angle = acos(fCos);

	if(Abs(angle) < constant::epsilon)
		return lhs;

	float sin = Sin(angle);
	float iSin = 1.0f / sin;
	float coeff0 = Sin((1.0f-fT) * angle) * iSin;
	float coeff1 = Sin(fT * angle) * iSin;
	return Normalize(lhs * coeff0 + rhs * coeff1);
}

const Vec3f RotateVec(const Quat q, const Vec3f v) {
	float nx = q.w * v.x + q.y * v.z - v.y * q.z;
	float ny = q.w * v.y + q.z * v.x - v.z * q.x;
	float nz = q.w * v.z + q.x * v.y - v.x * q.y;
	float nw = q.x * v.x + q.y * v.y + v.z * q.z;

	float nnx = nw * q.x + nx * q.w - ny * q.z + q.y * nz;
	float nny = nw * q.y + ny * q.w - nz * q.x + q.z * nx;
	float nnz = nw * q.z + nz * q.w - nx * q.y + q.x * ny;

	return Vec3f(nnx, nny, nnz);
}

const Vec3q RotateVec(const Quat q, Vec3q v) {
	floatq nx = q.w * v.x + q.y * v.z - v.y * q.z;
	floatq ny = q.w * v.y + q.z * v.x - v.z * q.x;
	floatq nz = q.w * v.z + q.x * v.y - v.x * q.y;
	floatq nw = q.x * v.x + q.y * v.y + v.z * q.z;

	floatq nnx = nw * q.x + nx * q.w - ny * q.z + q.y * nz;
	floatq nny = nw * q.y + ny * q.w - nz * q.x + q.z * nx;
	floatq nnz = nw * q.z + nz * q.w - nx * q.y + q.x * ny;
	return Vec3q(nnx, nny, nnz);
}

