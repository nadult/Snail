#ifndef RTRACER_SCENE_H
#define RTRACER_SCENE_H

#include "rtbase.h"
#include "light.h"
#include "shading.h"
#include "context.h"
#include "loader.h"


template <class Vec,class Container,class integer>
Vec FlatNormals(const Container &objects,const integer &objId)  {
	typedef typename Vec::TScalar real;
	typedef typename Container::value_type Object;

	const Object *obj0=&objects[objId[0]];
	Vec nrm(obj0->Nrm());

	if(ForAny(integer(objId[0])!=objId)) for(int n=1;n<ScalarInfo<real>::multiplicity;n++) {
		const Object *objN=&objects[objId[n]];
		if(objN!=obj0) {
			Vec newNrm(objN->Nrm());
			nrm=Condition(ScalarInfo<real>::ElementMask(n),newNrm,nrm);
		}
	}

	return nrm*RSqrt(nrm|nrm);
}


template <class Container,class integer,class Vec>
Vec GouraudNormals(const Container &objects,const ShadingDataVec &shadingData,const integer &objId,const Vec &rayOrig,const Vec &rayDir)  {
	typedef typename Vec::TScalar real;
	typedef typename Container::value_type Object;

	const Object *obj0=&objects[objId[0]];
	Vec nrm; {
		real u,v; obj0->Barycentric(rayOrig,rayDir,u,v);
		const ShadingData &data=shadingData[objId[0]];
		nrm=Vec(data.nrm[0])*(real(1.0f)-u-v)+Vec(data.nrm[2])*u+Vec(data.nrm[1])*v;
	}

	if(ForAny(integer(objId[0])!=objId)) for(int n=1;n<ScalarInfo<real>::multiplicity;n++) {
		const Object *objN=&objects[objId[n]];
		if(objN!=obj0) {
			Vec newNrm; {
				real u,v; objN->Barycentric(rayOrig,rayDir,u,v);
				const ShadingData &data=shadingData[objId[n]];
				newNrm=Vec(data.nrm[0])*(real(1.0f)-u-v)+Vec(data.nrm[2])*u+Vec(data.nrm[1])*v;
			}
			nrm=Condition(ScalarInfo<real>::ElementMask(n),newNrm,nrm);
		}
	}

	return nrm;
}

template <class Scene,class Group,class Selector>
void TraceLight(TracingContext<Scene,Group,Selector> &c,const Light &light) {
	typedef typename Vec3q::TScalar real;
	typedef typename Vec3q::TBool boolean;

	Vec3p lightPos=light.pos;

	Vec3q fromLight[Selector::size];
	real lightDist[Selector::size];

	for(int i=0;i<c.selector.Num();i++) {
		int q=c.selector.Idx(i);

		Vec3q lightVec=(c.position[q]-Vec3q(lightPos));
		lightDist[q]=Sqrt(lightVec|lightVec);
		fromLight[q]=lightVec*Inv(lightDist[q]);
	}

	RaySelector<Selector::size> lsel(c.selector);
	real dot[Selector::size];

	for(int i=0;i<lsel.Num();i++) {
		int q=lsel[i];
		dot[q]=c.normal[q]|fromLight[q]; {
			if(lsel.DisableWithMask(i,dot[q]<=Const<real,0>())) { i--; continue; }
			dot[q]=Condition(lsel.Mask(i),dot[q]);
		}
	}

	if(!lsel.Num()) return;

	real tDst[Selector::size]; {
		Vec3q lPos(lightPos.x,lightPos.y,lightPos.z);
		RayGroup<Group::size,1,0> tGroup(fromLight,&lPos);

		for(int i=0;i<lsel.Num();i++) {
			int q=lsel[i];
			tDst[q]=lightDist[q]*0.99999f;
		}

		c.scene.Traverse(tGroup,lsel,Output<otShadow,floatq,i32x4>(tDst,0,&c.stats));
	}

	Vec3q lightColor(light.color);
	for(int i=0;i<lsel.Num();i++) {
		int q=lsel.Idx(i);

		if(lsel.DisableWithMask(i,lightDist[q]-tDst[q]>lightDist[q]*0.0001f)) { i--; continue; }
		c.light[q]+=Condition(lsel.Mask(i),ShadeLight(lightColor,dot[q],lightDist[q]));
	}
}
template <class AccStruct>
class TScene
{
public:
	typedef typename AccStruct::Object Object;

	TScene(const char *modelFile);

	void Animate();
	void AddLight(Vec3f pos,Vec3f col);
	void AddSoftLight(Vec3f pos,Vec3f col,Vec3f dens,int dx,int dy,int dz);

	template <class Output,class Group,class Selector>
	void Traverse(Group &group,const Selector &sel,const Output &out) const {
		//tree.TraverseMonoGroup(group,sel,out);
		tree.TraverseOptimized(group,sel,out);
	}

	template <class Group,class Selector,bool primary>
	void RayTrace(TracingContext<TScene<AccStruct>,Group,Selector> &c) const NOINLINE;

	template <class Group,class Selector>
	void RayTracePrimary(TracingContext<TScene<AccStruct>,Group,Selector> &c) const { RayTrace<Group,Selector,1>(c); }
	template <class Group,class Selector>
	void RayTraceSecondary(TracingContext<TScene<AccStruct>,Group,Selector> &c) const { RayTrace<Group,Selector,0>(c); }

	bool lightsEnabled;
	vector<Light> lights;
	ShadingDataVec shadingData;
	AccStruct tree;
};

template <class Scene,class Group,class Selector>
void TraceReflection(TracingContext<Scene,Group,Selector> &c) {
	typedef typename Vec3q::TScalar real;
	typedef typename Vec3q::TBool boolean;

	Vec3q reflDir[Selector::size];

	for(int i=0;i<c.selector.Num();i++) {
		int q=c.selector.Idx(i);
		reflDir[q]=Reflect(c.RayDir(q),c.normal[q]);
	}

	typedef RayGroup<Group::size> TGroup;
	TracingContext<Scene,TGroup,Selector> rc(c.scene,TGroup(reflDir,c.position),c.selector);
	rc.options=c.options;
	rc.options.reflections--;

	c.scene.RayTraceSecondary(rc);
	c.stats.Update(rc.stats);

	for(int i=0;i<c.selector.Num();i++) {
		int q=c.selector.Idx(i);
		c.color[q]=Condition(c.selector.Mask(i),
						c.color[q]*Const<real,8,10>()+rc.color[q]*Const<real,2,10>(),
						c.color[q]);
	}
}

template <class AccStruct> template <class Group,class Selector,bool primary>
void TScene<AccStruct>::RayTrace(TracingContext<TScene<AccStruct>,Group,Selector> &c) const {
	typedef typename Vec3q::TScalar real;
	typedef typename Vec3q::TBool boolean;
	const floatq maxDist=100000.0f;

	for(int i=0;i<c.selector.Num();i++) {
		int q=c.selector.Idx(i);
		InitializationShader(c,q,maxDist);
	}

	c.density=0.5f;
	Traverse(c.rays,c.selector,Output<primary?otPrimary:otNormal,floatq,i32x4>(c));

	for(int i=0;i<c.selector.Num();i++) {
		int q=c.selector.Idx(i);
		const Object *obj=&tree.objects[0];

		if(c.selector.DisableWithMask(i,c.distance[q]>=maxDist)) {
			c.color[q]=Vec3q(Const<real,0>());
			i--; continue;
		}

		i32x4b imask(c.selector.Mask(i));

		c.position[q]=c.RayDir(q)*c.distance[q]+c.RayOrigin(q);
		if(c.options.shadingMode==smGouraud)
			c.normal[q]=GouraudNormals(tree.objects,shadingData,Condition(imask,c.objId[q]),c.RayOrigin(q),c.RayDir(q));
		else
			c.normal[q]=FlatNormals<Vec3q>(tree.objects,Condition(imask,c.objId[q]));
	
		SimpleLightingShader(c,q);
	}

	if(lightsEnabled) for(int n=0;n<lights.size();n++)
		TraceLight(c,lights[n]);

	if(c.options.reflections>0&&c.selector.Num())
		TraceReflection(c);

	if(lights.size()&&lightsEnabled)
		for(int i=0;i<c.selector.Num();i++) {
		int q=c.selector[i];
		c.color[q]=Condition(c.selector.Mask(i),c.color[q]*c.light[q]);
	}
	else for(int i=0;i<c.selector.Num();i++) {
		int q=c.selector[i];
		c.color[q]=Condition(c.selector.Mask(i),c.color[q]);
	}

	if(c.options.rdtscShader)
		for(int q=0;q<Selector::size;q++)
			StatsShader(c,q);
}


#include "scene.inl"

#endif

