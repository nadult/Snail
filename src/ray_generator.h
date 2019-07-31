#pragma once

#include "rtbase_math.h"

/*
extern const float bestCandidateSamples[4096][5];

class BestCandidateSampler {
public:
	const Vec2f operator()(int x,int y) const {
		int offset = (x & 63) + (y & 63) * 64;
		return Vec2f(	float(x) + bestCandidateSamples[offset][0],
						float(y) + bestCandidateSamples[offset][1] );
	}
};
*/

struct GridSampler {
	const Vec2f operator()(int x, int y) const { return Vec2f(x, y); }
};

/*!
	Generates primary ray packets
*/
class RayGenerator
{
public:
	RayGenerator(int level, int w, int h, float planeDist, Vec3f right, Vec3f up, Vec3f front);

	/*
		Rays are grouped in recursive Z pattern:
		(something like Hilbert curve)
		0  1  4  5  16 17 20 21
		2  3  6  7  18 19 22 23
		8  9  12 13 24 25 28 29
		10 11 14 15 26 27 30 31
		32 ...

		0:  x,y
		1:  x-1,y
		2:  x,y-1
		3:  x-1,y-1
	*/
	void Generate(int pw, int ph, int x, int y, Vec3q *out) const;

	/*!
		Decomposes group of ray output data from
		recursive zig pattern:

		special case for level 0:
		0 1		->	0 1
		2 3			2 3

		level>0:
		0 1  4 5		0 1 2 3
		2 3  6 7	->  4 5 6 7
		. .  . .		. . . .
		. .  . .		. . . .
	*/
	void Decompose(const Vec3q *in, Vec3q *out) const;
private:
	void Generate(int level, int pw, int ph, int x, int y, Vec3q *out) const;

	Vec3f tright, tup;
	Vec3q txyz;

	// invW is multiplied by ratio (w/h)
	float w, h, invW, invH, planeDist;
	int level;
};
