#ifndef RTRACER_SCENE_H
#define RTRACER_SCENE_H

#include "rtbase.h"
#include "shading.h"
#include "context.h"
#include <boost/type_traits/function_traits.hpp>
#include <boost/bind.hpp>

using boost::bind;
using boost::function_traits;


template <class Object,class integer,class Vec>
Vec ExtractNormals(const vector<Object> &objects,const integer &objId,const Vec &position)  {
	typedef typename Vec::TScalar real;

	const Object *obj0=&objects[objId[0]];
	Vec nrm=obj0->Normal(position);

	for(int n=1;n<ScalarInfo<real>::multiplicity;n++) {
		const Object *objN=&objects[objId[n]];
		if(objN!=obj0) {
			Vec newNrm=objN->Normal(position);
			nrm=Condition(ScalarInfo<real>::ElementMask(n),newNrm,nrm);
		}
	}

	return nrm;
}

template <class Scene,class Group,class Vec,class integer>
void TraceLight(TracingContext<Scene,Group,Vec,integer> &c,const Light &light) NOINLINE;

template <class Scene,class Group,class Vec,class integer>
void TraceReflection(TracingContext<Scene,Group,Vec,integer> &c) NOINLINE;


template <class Scene,class Group,class Vec,class integer>
void TraceLight(TracingContext<Scene,Group,Vec,integer> &c,const Light &light) {
	typedef typename Vec::TScalar real;
	typedef typename Vec::TBool boolean;

	Vec3p lightPos=light.pos;

	Vec3q fromLight[Group::size];
	real lightDist[Group::size];

	for(int i=0;i<c.selector.Num();i++) {
		int q=c.selector.Idx(i);

		Vec3q lightVec=(c.position[q]-Vec3q(lightPos));
		lightDist[q]=Sqrt(lightVec|lightVec);
		fromLight[q]=lightVec*Inv(lightDist[q]);
	}

	RaySelector<Group::size> lsel(c.selector);
	real dot[Group::size];

	for(int i=0;i<lsel.Num();i++) {
		int q=lsel[i];
		dot[q]=c.normal[q]|fromLight[q]; {
			if(lsel.DisableWithMask(i,dot[q]<=Const<real,0>())) { i--; continue; }
			dot[q]=Condition(lsel.Mask(q),dot[q]);
		}
	}

	if(!lsel.Num()) return;

	real tDst[Group::size]; {
		Vec3q lPos(lightPos.x,lightPos.y,lightPos.z);
		RayGroup<Group::recLevel,1> tGroup(fromLight,&lPos);

		for(int i=0;i<lsel.Num();i++) {
			int q=lsel[i];
			tDst[q]=lightDist[q]+1000.0f;
		}

		c.scene.Traverse(tGroup,lsel,ShadowOutput<real,integer>(tDst,&c.stats),0);
	}

	Vec lightColor=light.color;
	for(int i=0;i<lsel.Num();i++) {
		int q=lsel.Idx(i);

		if(lsel.DisableWithMask(i,lightDist[q]-tDst[q]>lightDist[q]*0.0001f)) { i--; continue; }
		c.light[q]+=Condition(lsel.Mask(q),ShadeLight(lightColor,dot[q],lightDist[q]));
	}
}

template <class Scene,class Group,class Vec,class integer>
void TraceReflection(TracingContext<Scene,Group,Vec,integer> &c) {
	typedef typename Vec::TScalar real;
	typedef typename Vec::TBool boolean;

	Vec3q reflDir[Group::size];
	RayStore<Group::size,0> store(reflDir,c.position);

	for(int i=0;i<c.selector.Num();i++) {
		int q=c.selector.Idx(i);
		reflDir[q]=Reflect(c.rayGroup.Dir(q),c.normal[q]);
	}

	TracingContext<Scene,RayGroup<Group::recLevel,0>,Vec,integer> rc(c.scene,store,c.selector);
	rc.options=c.options;
	rc.options.reflections--;

	c.scene.RayTrace(rc,0);
	c.stats.Update(rc.stats);

	for(int i=0;i<c.selector.Num();i++) {
		int q=c.selector.Idx(i);
		c.color[q]=Condition(c.selector.Mask(q),
						c.color[q]*Const<real,8,10>()+rc.color[q]*Const<real,2,10>(),
						c.color[q]);
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

	template <class Output,class Group>
	void Traverse(Group &group,const RaySelector<Group::size> &sel,const Output &out,bool primary=1) const {
		//tree.TraverseMonoGroup(group,sel,out);
		tree.TraverseOptimized(group,sel,out,primary);
	}

	template <class Group,class Vec,class integer>
	void RayTrace(TracingContext<TScene<AccStruct>,Group,Vec,integer> &c,bool primary=1) const NOINLINE;
	vector<Light> lights;
	AccStruct tree;
};


template <class AccStruct> template <class Group,class Vec,class integer>
void TScene<AccStruct>::RayTrace(TracingContext<TScene<AccStruct> ,Group,Vec,integer> &c,bool primary) const {
	typedef typename Vec::TScalar real;
	typedef typename Vec::TBool boolean;
	const floatq maxDist=100000.0f;

	for(int i=0;i<c.selector.Num();i++) {
		int q=c.selector.Idx(i);
		InitializationShader(c,q,maxDist);
	}

	c.density=0.5f;
	Traverse(c.rayGroup,c.selector,NormalOutput<real,integer>(c),primary);

	for(int i=0;i<c.selector.Num();i++) {
		int q=c.selector.Idx(i);
		const Object *obj=&tree.objects[0];

		if(c.selector.DisableWithMask(i,c.distance[q]>=maxDist)) {
			c.color[q]=Vec(Const<real,0>());
			i--; continue;
		}

		i32x4b imask(c.selector.Mask(q));

		c.position[q]=c.rayGroup.Dir(q)*c.distance[q]+c.rayGroup.Origin(q);
		c.normal[q]=ExtractNormals(tree.objects,Condition(imask,c.objId[q]),c.position[q]);
		
		SimpleLightingShader(c,q);
	}

	for(int n=0;n<lights.size();n++)
		TraceLight(c,lights[n]);

	if(c.options.reflections>0&&c.selector.Num())
		TraceReflection(c);

	if(lights.size())
		for(int i=0;i<c.selector.Num();i++) {
		int q=c.selector[i];
		c.color[q]=Condition(c.selector.Mask(q),c.color[q]*c.light[q]);
	}
	else for(int i=0;i<c.selector.Num();i++) {
		int q=c.selector[i];
		c.color[q]=Condition(c.selector.Mask(q),c.color[q]);
	}

//	if(c.options.rdtscShader)
//		for(int q=0;q<Group::size;q++)
//			c.color[q].x=c.color[q].y=0.0f;
	if(c.options.rdtscShader)
		for(int q=0;q<Group::size;q++)
			StatsShader(c,q);
}


#include "scene.inl"

#endif

