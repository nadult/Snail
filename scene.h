#ifndef RTRACER_SCENE_H
#define RTRACER_SCENE_H

#include "rtbase.h"
#include "light.h"
#include "shading.h"
#include "context.h"
#include "formats/loader.h"

template <int size_>
class Result {
public:
	enum { size=size_ };
	Vec3q color[size];
	TreeStats<1> stats;
};

template <class Container,class integer,class Vec>
Vec GouraudNormals(const Container &objects,const ShadingDataVec &shadingData,const integer &objId,const Vec &rayOrig,const Vec &rayDir)  {
	typedef typename Vec::TScalar real;
	typedef typename Container::value_type Element;

	const Element *obj0=&objects[objId[0]];
	Vec nrm; {
		real u,v; obj0->Barycentric(rayOrig,rayDir,u,v);
		const ShadingData &data=shadingData[objId[0]];
		nrm=Vec(data.nrm[0])*(real(1.0f)-u-v)+Vec(data.nrm[2])*u+Vec(data.nrm[1])*v;
	}

	if(ForAny(integer(objId[0])!=objId)) for(int n=1;n<ScalarInfo<real>::multiplicity;n++) {
		const Element *objN=&objects[objId[n]];
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

template <int packetSize,class AccStruct,class Selector>
Result<packetSize> TraceLight(const AccStruct &tree,const Selector &inputSel,const Vec3q *position,
								const Vec3q *normal,const Light &light,int idx) {
	enum { size=packetSize };
	Vec3p lightPos=light.pos;

	Vec3q fromLight[packetSize];
	floatq lightDist[packetSize];

	Result<size> out;
	RaySelector<size> sel=inputSel;

	for(int q=0;q<size;q++) {
		if(!sel[q]) continue;

		Vec3q lightVec=(position[q]-Vec3q(lightPos));
		f32x4b close=LengthSq(lightVec)<0.0001f;
		lightVec=Condition(close,Vec3q(0.0f,1.0f,0.0f),lightVec);

		lightDist[q]=Sqrt(lightVec|lightVec);
		fromLight[q]=lightVec*Inv(lightDist[q]);
		out.color[q]=Vec3q(0.0f,0.0f,0.0f);
	}

	floatq dot[packetSize];

	enum { shadows=1 };

	for(int q=0;q<size;q++) {
		dot[q]=normal[q]|fromLight[q];
		f32x4b mask=dot[q]>0.0f;
		sel[q]&=ForWhich(mask);
		dot[q]=Condition(mask,dot[q]);
	}

	Isct<f32x4,packetSize,AccStruct::isctFlags|isct::fShadow|isct::fMaxDist> hit; if(shadows) {
		Vec3q lPos(lightPos.x,lightPos.y,lightPos.z);
		RayGroup<packetSize,isct::fShOrig|isct::fInvDir|isct::fMaxDist|isct::fShadow> rays(&lPos,fromLight);

		for(int q=0;q<size;q++) rays.maxDist.Set(q,lightDist[q]*0.99999f);
		hit=tree.TraversePacket(rays,sel);
		out.stats=hit.Stats();
	}

	Vec3q lightColor(light.color);
	for(int q=0;q<size;q++) {
		if(!sel[q]) continue;

		if(shadows) {
			f32x4b mask=lightDist[q]-hit.Distance(q)<=lightDist[q]*0.0001f;
			out.color[q]=Condition(mask,ShadeLight(lightColor,dot[q],lightDist[q]));
		}
		else out.color[q]=ShadeLight(lightColor,dot[q],lightDist[q]);
	}

	return out;
}

#include "sampling/point_sampler.h"
#include "sampling/point_sampler_dxt.h"
#include "material.h"

extern shading::SimpleMaterial<sampling::PointSampler> material[8];

template <class AccStruct,int flags,int packetSize,class Selector>
Result<packetSize> RayTrace(const AccStruct &tree,const RayGroup<packetSize,flags> &rays,
							const Selector &inputSelector) {
	enum { primary=0, size=Selector::size };
	
	typedef typename Vec3q::TScalar real;
	typedef typename Vec3q::TBool boolean;
	const floatq maxDist=100000.0f;

	Isct<f32x4,size,AccStruct::isctFlags> hit=tree.TraversePacket(rays,inputSelector);
	RaySelector<size> selector=inputSelector;

	shading::Sample samples[size];

	if(gVals[4]) { //no shading
		Result<size> result;
		result.stats=hit.Stats();
		for(int q=0;q<size;q++)
			result.color[q]=Vec3q((Condition(hit.Distance(q)>maxDist,0.0f,hit.Distance(q)))*0.03f,0.0f,0.0f);
		return result;
	}

	for(int q=0;q<size;q++) {
		shading::Sample &s=samples[q];

		f32x4b mask=hit.Distance(q)<maxDist&&selector.SSEMask(q);
		selector[q]=ForWhich(mask);
		s.mask=mask;

		i32x4b imask(mask);
		int invBitMask=~ForWhich(imask);

		i32x4 object=Condition(imask,hit.Object(q),i32x4(0));
		i32x4 element=AccStruct::isctFlags&isct::fElement?Condition(imask,hit.Element(q),i32x4(0)):i32x4(0);

		s.matId=Condition(imask,(object&7)+i32x4(1));
		s.color=Vec3q(0.0f,0.0f,0.0f);

		if(!selector[q]) continue;

		s.position=rays.Dir(q)*hit.Distance(q)+rays.Origin(q);

		int obj0=object[0],elem0=element[0];
		if(i32x4(imask)[0]) {
			const ShTriangle &shTri=tree.GetShElement(obj0,elem0);
			Vec3q bar=tree.Barycentric(rays.Origin(q),rays.Dir(q),obj0,elem0);

			s.texCoord=	Vec2q(shTri.uv[0].x,shTri.uv[0].y)*bar.x+
						Vec2q(shTri.uv[1].x,shTri.uv[1].y)*bar.y+
						Vec2q(shTri.uv[2].x,shTri.uv[2].y)*bar.z;
			Broadcast(Maximize(s.texCoord)-Minimize(s.texCoord),s.texDiff);

			s.normal=Vec3q(shTri.nrm[0])*bar.x+Vec3q(shTri.nrm[1])*bar.y+Vec3q(shTri.nrm[2])*bar.z;
		}

		for(int k=1;k<4;k++) {
			int obj=object[k],elem=element[k];
			if(invBitMask&(1<<k)||(obj==obj0&&elem==elem0)) continue;

			const ShTriangle &shTri=tree.GetShElement(obj,elem);

			Vec3q bar=tree.Barycentric(rays.Origin(q),rays.Dir(q),obj,elem);

			Vec2q tex=Vec2q(shTri.uv[0].x,shTri.uv[0].y)*bar.x+
					  Vec2q(shTri.uv[1].x,shTri.uv[1].y)*bar.y+
					  Vec2q(shTri.uv[2].x,shTri.uv[2].y)*bar.z;

			Vec2f diff=Maximize(tex)-Minimize(tex);
			s.texDiff.x[k]=diff.x; s.texDiff.y[k]=diff.y;
			s.texCoord.x[k]=tex.x[k]; s.texCoord.y[k]=tex.y[k];
			Vec3f nrm=shTri.nrm[0]*bar.x[k]+shTri.nrm[1]*bar.y[k]+shTri.nrm[2]*bar.z[k];
			
			s.normal.x[k]=nrm.x; s.normal.y[k]=nrm.y; s.normal.z[k]=nrm.z;
		}
	}

	enum { matBlock=4 };
	for(int b=0;b<size;b+=matBlock) {
		const int endB=b+matBlock;

		i32x4 matIds[matBlock]={i32x4(0),i32x4(0),i32x4(0),i32x4(0)};
		uint matIdCount=0;

		for(int q=b;q<endB;q++) {
			int invBitMask=~ForWhich(samples[q].mask);
			const i32x4 &matId=samples[q].matId;

			for(int k=0;k<4;k++) {
				if(invBitMask&(1<<k)) continue;

				int id=matId[k];
				i32x4 kMatId(id);
				if(ForAll(matIds[0]!=kMatId&&matIds[1]!=kMatId&&matIds[2]!=kMatId&&matIds[3]!=kMatId)) {
					matIds[matIdCount/4][matIdCount&3]=id;
					matIdCount++;
				}
			}
		}

		for(int n=0;n<matIdCount;n++) {
			uint id=matIds[n/4][n&3];
			i32x4 id4(id);

			const shading::SimpleMaterial<sampling::PointSampler> &mat=material[id-1];
			mat.PreShade(samples,rays,b,matBlock);
		}
	}

/*	enum { lightsEnabled=0 };

	if(lightsEnabled) {
		float pos=float(gVals[5])*0.01f;
		Light lights[]={
			Light(RotateY(pos)*Vec3f(0,1.5f,0),Vec3f(5,5,3)*20.5f),
		//	Light(RotateY(pos*0.7f)*Vec3f(1,1.5f,0),Vec3f(1,5,2)*20.5f),
		//	Light(Vec3f(0,1.5f,1),Vec3f(5,1,1)*0.5f),
					};

		for(int q=0;q<size;q++) light[q]=Vec3q(0.2f,0.2f,0.2f);

		for(int n=0;n<sizeof(lights)/sizeof(Light);n++) {
			Result<size> lightResult=TraceLight<size>(tree,selector,position,normals,lights[n],n);
			for(int q=0;q<size;q++) light[q]+=lightResult.color[q];
			result.stats+=lightResult.stats;
		}
	}*/
	
	Result<size> result;
	result.stats=hit.Stats();
	
	for(int q=0;q<size;q++) {
	//	material.PostShade(samples[q]);
		result.color[q]=samples[q].color;
	}


/*	if(gVals[2]&&flags&isct::fPrimary) {
		Result<size> refl=TraceReflection(tree,rays,selector,position,normals);
		result.stats+=refl.stats;

		for(int q=0;q<size;q++) {
			floatq str=0.2f;
			result.color[q]=Condition(selector.SSEMask(q),
					result.color[q]*(floatq(1.0f)-str)+refl.color[q]*str,result.color[q]);
		}
	}*/

	return result;
}

template <class AccStruct,int flags,int size,class Selector>
Result<size> TraceReflection(const AccStruct &tree,const RayGroup<size,flags> &rays,const Selector &selector,
								const Vec3q *position,const Vec3q *normal) {
	Vec3q reflDir[size];

	for(int q=0;q<size;q++)
		reflDir[q]=Reflect(rays.Dir(q),normal[q]);
	RayGroup<size,isct::fInvDir> tRays(position,reflDir);

	return RayTrace(tree,tRays,selector);
}

#endif

