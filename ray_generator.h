#include "rtracer.h"

/*!
	Generates primary ray packets
*/
class RayGenerator
{
public:
	RayGenerator(int level,int w,int h,float pd);

	/*
		Rays are grouped in recursive Z pattern:
		(something like Hilbert curve)
		0  1  4  5  16 17 ...
		2  3  6  7  18 19
		8  9  12 13 
		10 11 14 15
		32 ...

		0:  x,y
		1:  x-1,y
		2:  x,y-1
		3:  x-1,y-1
	*/
	void Generate(int pw,int ph,int x,int y,Vec3q *out);

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
	void Decompose(const Vec3q *in,Vec3q *out);
private:
	void Generate(int level,int pw,int ph,int x,int y,Vec3q *out);

	Vec3q addVec;
	// invW is multiplied by ratio (w/h)
	float invW,invH,planeDist;
	int tLevel;
};
