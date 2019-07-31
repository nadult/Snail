#pragma once

#include "rtbase.h"

class AxisAngle
{
public:
	AxisAngle() { }
	AxisAngle(const Vec3f &axis, float angle) : axis(Normalize(axis)), radians(angle) { }
//	inline operator const Matrix3() const { return Rotate(Axis(), Angle()); }

	float Angle() const { return radians; }
	const Vec3f Axis() const { return axis; }

protected:
	Vec3f axis;
	float   radians;
};

class Quat : public Vec4f
{
public:
	Quat() { }
	Quat(const Quat&q) : Vec4f(q) { }
	Quat& operator=(const Quat &q) {
		Vec4f::operator=(Vec4f(q));
		return *this;
	}
	explicit Quat(const Vec4f&v) : Vec4f(v) { }
	explicit Quat(const Matrix<Vec4f>&);
	Quat(const AxisAngle&);
	Quat(float yaw, float pitch, float roll);

	operator const AxisAngle() const;

	const Quat &operator+=(const Quat &a)	{ Vec4f::operator+=(a); return *this; }
	const Quat &operator-=(const Quat &a)	{ Vec4f::operator-=(a); return *this; }
	const Quat &operator*=(const float &a)	{ Vec4f::operator*=(a); return *this; }
	const Quat &operator/=(const float &a)	{ Vec4f::operator/=(a); return *this; }

	float operator|(const Quat &v) const	{ return x * x + y * y + z * z + w * w; }
	const Quat operator-() const			{ return Quat(Vec4f::operator-()); }
	const Quat operator~() const			{ return Quat(Vec4f(-x, -y, -z, w)); }

	const Quat& operator*=(const Quat&);
	const Quat &operator*=(const Vec3f&);
};

const Quat Inverse(const Quat&);

inline const Quat operator +(const Quat lhs, const Quat rhs)	{ return Quat(lhs) += rhs; }
inline const Quat operator -(const Quat lhs, const Quat rhs)	{ return Quat(lhs) -= rhs; }
inline const Quat operator *(const Quat lhs, const float rhs)	{ return Quat(lhs) *= rhs; }
inline const Quat operator *(const Quat lhs, const Quat rhs)	{ return Quat(lhs) *= rhs; }
inline const Quat operator *(const Quat lhs, const Vec3f rhs)	{ return Quat(lhs) *= rhs; }
inline const Quat operator /(const Quat lhs, const float rhs)	{ return Quat(lhs) /= rhs; }
const Quat operator *(const Vec3f&, const Quat&);

const Vec3f RotateVec(const Quat quat, const Vec3f);
const Vec3q RotateVec(const Quat quat, const Vec3q);
