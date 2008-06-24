#ifndef RTRACER_SHADING_H
#define RTRACER_SHADING_H

#include "rtbase.h"
#include "context.h"

template <class Context>
void InitializationShader(Context &c,uint i,const typename Context::real &maxDist) {
	Vec3q ambient; Broadcast(Vec3f(0.2,0.2,0.2),ambient);
	c.distance[i]=maxDist;
	c.color[i]=Const<f32x4,0>();
	c.light[i]=ambient;
}

template <class Context>
void SimpleLightingShader(Context &c,uint i) {
	c.color[i]=c.RayDir(i)|c.normal[i];
//	c.color[i]*=c.color[i];
//	c.color[i]*=c.color[i];
}

template <class Context>
void ReflectionShader(Context &c,uint i) {
	typedef typename Context::real real;
	Vec3q refl=Reflect(c.RayDir(i),c.normal[i]);

	c.color[i].x=Condition(refl.x>Const<real,0>(),Const<real,8,12>(),Const<real,2,12>());
	c.color[i].y=Condition(refl.y>Const<real,0>(),Const<real,8,12>(),Const<real,2,12>());
	c.color[i].z=Condition(refl.z>Const<real,0>(),Const<real,8,12>(),Const<real,2,12>());
}


template <class Context>
void RayDirectionShader(Context &c,uint i) {
	Vec3q dir=c.RayDir(i);

	c.color[i].x=Condition(dir.x<Const<floatq,0>(),Const<floatq,1,2>(),Const<floatq,1>());
	c.color[i].y=Condition(dir.y<Const<floatq,0>(),Const<floatq,1,2>(),Const<floatq,1>());
	c.color[i].z=Condition(dir.z<Const<floatq,0>(),Const<floatq,1,2>(),Const<floatq,1>());
}

template <class Context>
void DistanceShader(Context &c,uint i) {
	typedef typename Context::real real;
	real dist=c.distance[i];

	c.color[i].x=Abs(Const<real,500>()/dist);
	c.color[i].y=Abs(Const<real,200>()/dist);
	c.color[i].z=Abs(Const<real,50>()/dist);

//	c.color[i].x=Abs(Const<real,1>()-dist*Const<real,1,50>());
//	c.color[i].y=Abs(Const<real,1>()-dist*Const<real,1,200>());
//	c.color[i].z=Abs(Const<real,1>()-dist*Const<real,1,1000>());
}

template <class Vec,class real>
Vec ShadeLight(const Vec &lightColor,const real &dot,const real &lightDist) {
	Vec out;
	real mul=Inv(lightDist*lightDist);
	real spec=dot*dot; spec=spec*spec; //spec*=spec; spec*=spec;
	out = ( 
			 lightColor*dot
			+Vec3q(lightColor.x,0.0f,0.0f)*spec
			)*mul;
	return out;
}

template <class Context>
void StatsShader(Context &c,uint i) {
	typedef typename Context::real real;

//	c.color[i].x*=0.1f;
//	c.color[i].y*=0.1f;
	c.color[i].x=float(c.stats.Intersects())*(0.05f/Context::size);
	c.color[i].y=float(c.stats.LoopIters())*(0.01f/Context::size);
	c.color[i].z=float(c.stats.Skips()*0.25f);
}
	


#endif

