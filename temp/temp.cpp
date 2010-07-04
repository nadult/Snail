#include <emmintrin.h>
#include <iostream>

class NiceVector {
public:
	NiceVector(const __m128 &v) :m(v) { }
	NiceVector(const NiceVector &v) :m(v.m) { }
	const NiceVector &operator=(const NiceVector &v) { m=v.m; return *this; }

	union {
		__m128 m;
		struct { float x,y,z,w; };
	};
};

class FastVector {
public:
	FastVector(float x,float y,float z,float w) :m(_mm_set_ps(w,z,y,x)) { }
	FastVector(const __m128 &m) :m(m) { }
	FastVector(const FastVector &v) :m(v.m) { }
	const FastVector &operator=(const FastVector &v) { m=v.m; return *this; }
	NiceVector *operator->() { return (NiceVector*)this; }

	__m128 m;
};

int main() {
	FastVector vec(1,2,3,4);
	std::cout << vec->x << ' ' << vec->y << ' ' << vec->z << ' ' << vec->w << '\n';
	return 0;
}
