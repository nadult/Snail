#ifndef RTRACER_SHADING_H
#define RTRACER_SHADING_H

#include "rtbase.h"
#include "context.h"

template <int size,class Shader>
void Shade(const Shader &shader,Vec3q output[size]) {
	for(int q=0;q<size;q++)
		output[q]=shader[q];
}

template <int size,class Shader,template <int> class Selector>
void Shade(const Shader &shader,const Selector<size> &sel,Vec3q output[size]) {
	if(size>=4) for(int t=0;t<size/4;t+=4) {
		if(!sel.Mask4(t)) continue;

		if(sel[t+0]) output[t+0]=Condition(sel.SSEMask(t+0),shader[t+0]);
		if(sel[t+1]) output[t+1]=Condition(sel.SSEMask(t+1),shader[t+1]);
		if(sel[t+2]) output[t+2]=Condition(sel.SSEMask(t+2),shader[t+2]);
		if(sel[t+3]) output[t+3]=Condition(sel.SSEMask(t+3),shader[t+3]);
	}
	for(int q=(size/4)*4;q<size;q++)
		if(sel[q]) output[q]=Condition(sel.SSEMask(q),shader[q]);
}


template <class Context>
void InitializationShader(Context &c,uint i,const typename Context::real &maxDist) {
	Vec3q ambient; Broadcast(Vec3f(0.2,0.2,0.2),ambient);
	c.distance[i]=maxDist;
	c.color[i]=Const<f32x4,0>();
	c.light[i]=ambient;
	c.objId[i]=c.elementId[i]=i32x4(0);
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


template <int size>
struct StatsShader {
	StatsShader(const TreeStats<1> &tStats) :stats(tStats) { }
	Vec3q operator[](int) const {
		return Vec3q(
			float(stats.GetIntersects())*(0.04f/size),0.0f,
		//	float(stats.GetLoopIters())*(0.02f/size),
			float(stats.GetSkips()*0.25f) );
	}
	const TreeStats<1> &stats;
};

struct DistanceShader {
	DistanceShader(const floatq *dist) :distance(dist) { }
	Vec3q operator[](int q) const {
		floatq idist=Inv(distance[q]);
		return Vec3q( Abs(idist*floatq(1.0f)), Abs(idist*floatq(0.1f)), Abs(idist*floatq(0.01f)) );
	}
	const floatq *distance;
};

#endif

