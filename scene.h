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
			boolean mask=dot[q]<=Const<real,0>();
			if(ForAll(mask)) {
				lsel.Disable(i--);
				continue;
			}
			dot[q]=Condition(!mask,dot[q]);
		}
	}

	real tDst[Group::size]; {
		Vec3q lPos(lightPos.x,lightPos.y,lightPos.z);
		RayGroup<Group::recLevel,1> tGroup(fromLight,&lPos);
		c.scene.Traverse(tGroup,lsel,Const<real,10000>(),ShadowOutput<real,integer>(c));
	}

	Vec lightColor=light.color;
	for(int i=0;i<lsel.Num();i++) {
		int q=lsel.Idx(i);

		boolean mask=Abs(tDst[q]-lightDist[q])>Const<real,1,1000>();
		if(ForAll(mask)) // wszystkie punkty zasloniete
			continue;	

		Vec col=Condition(!mask,ShadeLight(lightColor,dot[q],lightDist[q]));
		c.light[q]+=col;
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

	c.scene.RayTrace(rc);
	c.stats.Update(rc.stats);

	for(int i=0;i<c.selector.Num();i++) {
		int q=c.selector.Idx(i);
		c.color[q]=c.color[q]*Const<real,8,10>()+rc.color[q]*Const<real,2,10>();
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
	void Traverse(Group &group,const RaySelector<Group::size> &sel,const floatq &maxD,const Output &out) const {
		tree.TraverseOptimized(group,sel,maxD,out);
	}

	template <class Scene,class Group,class Vec,class integer>
	void RayTrace(TracingContext<Scene,Group,Vec,integer> &c) const {
		typedef typename Vec::TScalar real;
		typedef typename Vec::TBool boolean;
		Group &group=c.rayGroup;

		Vec *nrm=c.normal,*colPos=c.position,*reflDir=c.reflectionDir,*accLight=c.light;
		boolean *tmask=c.hitMask; real *dst=c.distance;
		integer *objId=c.objId;

		unsigned long long ticks=Ticks();
		const real maxDist=Const<real,10000>();

		Traverse(group,c.selector,maxDist,NormalOutput<real,integer>(c));

		for(int i=0;i<c.selector.Num();i++) {
			int q=c.selector.Idx(i);
			const Object *obj=&tree.objects[0];

			tmask[q]=dst[q]>=maxDist;
			if(ForAll(tmask[q])) {
				c.color[q]=Vec(Const<real,0>());
				c.selector.Disable(i--);
				continue;
			}

			colPos[q]=group.Dir(q)*dst[q]+group.Origin(q);
			nrm[q]=ExtractNormals(tree.objects,objId[q],colPos[q]);
			
			InitializationShader(c,q);
			SimpleLightingShader(c,q);
		//	RayDirectionShader(c,q);
		}

		for(int n=0;n<lights.size();n++)
			TraceLight(c,lights[n]);

		if(c.options.reflections>0&&c.selector.Num())
			TraceReflection(c);

		uint nInters=c.stats.Intersects();
		uint nLIters=c.stats.LoopIters();

		if(lights.size()) for(int i=0;i<c.selector.Num();i++) {
			int q=c.selector[i];
			c.color[q]=Condition(!c.hitMask[q],c.color[q]*c.light[q]);
		}
		else for(int i=0;i<c.selector.Num();i++) {
			int q=c.selector[i];
			c.color[q]=Condition(!c.hitMask[q],c.color[q]);
		}

		if(c.options.rdtscShader)
			for(int q=0;q<Group::size;q++)
				StatsShader(c,q);
	}

	vector<Light> lights;
	AccStruct tree;
};

#include "scene.inl"

#endif

