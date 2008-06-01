#ifndef RTBASE_H
#define RTBASE_H

#include <baselib.h>
#include <veclib.h>


using namespace baselib;
using namespace veclib;

typedef Vec2<float> Vec2f;
typedef Vec3<float> Vec3f;
typedef Vec4<float> Vec4f;
typedef pvec2f32	Vec2p;
typedef pvec3f32	Vec3p;
typedef pvec4f32	Vec4p;
typedef vec2f32x4	Vec2q;
typedef vec3f32x4	Vec3q;
typedef vec4f32x4	Vec4q;
typedef f32x4		floatq;
typedef i32x4		intq;

template <class Vec>
Vec Reflect(const Vec &ray,const Vec &nrm) {
	typename Vec::TScalar dot=(nrm|ray);
	return ray-nrm*(dot+dot);
}

inline float Maximize(const floatq &t) { return Max(Max(t[0],t[1]),Max(t[2],t[3])); }
inline float Minimize(const floatq &t) { return Min(Min(t[0],t[1]),Min(t[2],t[3])); }
inline Vec3p Maximize(const Vec3q &v) { return Vec3p(Maximize(v.x),Maximize(v.y),Maximize(v.z)); }
inline Vec3p Minimize(const Vec3q &v) { return Vec3p(Minimize(v.x),Minimize(v.y),Minimize(v.z)); }


template <int size>
class ObjectIdxBuffer
{
public:
	ObjectIdxBuffer() {
		for(int n=0;n<size;n++)
			indices[n]=i32x4(-1);
		last=0;
	}
	void Insert(u32 idx) {
		((u32*)indices)[last]=idx;
		last=(last+1)%(size*4);
	}
	bool Find(u32 idx) {
		i32x4 tidx(idx); i32x4b test;
		test=indices[0]==tidx;
		for(int n=1;n<size;n++)
			test=test||indices[n]==tidx;
		return ForAny(test);
	}
	i32x4 indices[size];
	uint last;
};

template <class T>
class Vector
{
public:
	typedef T value_type;

	Vector() :tab(0),count(0) { }

	template <class Container>
	Vector(const Container &obj) :tab(0) {
		Alloc(obj.size());
		for(int n=0;n<count;n++) tab[n]=obj[n];
	}
	Vector(const Vector &obj) :tab(0) {
		Alloc(obj.size());
		for(int n=0;n<count;n++) tab[n]=obj[n];
	}
	const Vector &operator=(const Vector &obj) {
		if(&obj==this) return *this;
		return operator=<Vector>(obj);
	}
	template <class Container>
	const Vector &operator=(const Container &obj) {
		Alloc(obj.size());
		for(int n=0;n<count;n++) tab[n]=obj[n];
		return *this;
	}
	~Vector() { Free(); }

	inline size_t size() const { return count; }
	inline const T &operator[](int n) const { return tab[n]; }
	inline T &operator[](int n) { return tab[n]; }

protected:
	void Alloc(size_t newS) { Free(); count=newS; tab=count?(T*)_mm_malloc(sizeof(T)*count,64):0; }
	void Free() { if(tab) _mm_free(tab); } 

	T *tab;
public:
	size_t count;
};

template <class T>
class TVector: public Vector<T> {
public:
	TVector(size_t res) { Vector<T>::Alloc(res); reserve=Vector<T>::count; }

	void push_back(const T &t) { Vector<T>::operator[](Vector<T>::count++)=t; }
	inline const T &operator[](int n) const { return Vector<T>::tab[n]; }
	inline T &operator[](int n) { return Vector<T>::tab[n]; }

private:
	int reserve;
};


#endif

