#ifndef RTRACER_SCENE_H
#define RTRACER_SCENE_H

#include "rtbase.h"
#include "light.h"
#include "shading.h"
#include "context.h"
#include "formats/loader.h"
#include "gfxlib_texture.h"

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

template <class Container>
Vec2q GouraudTexCoords(const Container &objects,const ShadingDataVec &shadingData,const i32x4 &objId,
						const Vec3q &rayOrig,const Vec3q &rayDir)  {
	typedef typename Vec3q::TScalar real;
	typedef typename Container::value_type Element;

	const Element *obj0=&objects[objId[0]];
	Vec2q nrm; {
		real u,v; obj0->Barycentric(rayOrig,rayDir,u,v);
		const ShadingData &data=shadingData[objId[0]];
		nrm=Vec2q(data.uv[0])*(real(1.0f)-u-v)+Vec2q(data.uv[2])*u+Vec2q(data.uv[1])*v;
	}

	if(ForAny(i32x4(objId[0])!=objId)) for(int n=1;n<ScalarInfo<real>::multiplicity;n++) {
		const Element *objN=&objects[objId[n]];
		if(objN!=obj0) {
			Vec2q newNrm; {
				real u,v; objN->Barycentric(rayOrig,rayDir,u,v);
				const ShadingData &data=shadingData[objId[n]];
				newNrm=Vec2q(data.uv[0])*(real(1.0f)-u-v)+Vec2q(data.uv[2])*u+Vec2q(data.uv[1])*v;
			}
			nrm=Condition(ScalarInfo<real>::ElementMask(n),newNrm,nrm);
		}
	}

	return nrm;
}


template <int packetSize,class AccStruct>
TreeStats<1> TraceLight(const AccStruct &tree,const floatq *dist,const Vec3q *position,const Vec3q *normal,
				Vec3q * __restrict__ out,const Light &light,ShadowCache &cache,int idx) {
	enum { size=packetSize };
	Vec3p lightPos=light.pos;

	Vec3q fromLight[packetSize];
	Vec3q fromLightInv[packetSize];
	floatq lightDist[packetSize];

	RaySelector<packetSize> sel;
	sel.SelectAll();

	for(int q=0;q<size;q++) {
		f32x4b mask=dist[q]<(1.0f/0.0f);
		sel[q]&=ForWhich(mask);
		if(!sel[q]) continue;

		Vec3q lightVec=(position[q]-Vec3q(lightPos));
		f32x4b close=LengthSq(lightVec)<0.0001f;
		lightVec=Condition(close,Vec3q(0.0f,1.0f,0.0f),lightVec);

		lightDist[q]=Sqrt(lightVec|lightVec);
		fromLight[q]=lightVec*Inv(lightDist[q]);
		fromLightInv[q]=VInv(fromLight[q]);
	}

	floatq dot[packetSize];

	for(int q=0;q<size;q++) {
		dot[q]=normal[q]|fromLight[q];
		f32x4b mask=dot[q]>0.0f;
		sel[q]&=ForWhich(mask);
		dot[q]=Condition(mask,dot[q]);
	}

	bool allDisabled=1;
	if(size>1) { for(int q=0;q<size/4&&allDisabled;q++) if(sel.Mask4(q)) allDisabled=0; }
	else  allDisabled=0;
	if(allDisabled) return TreeStats<1>();

	f32x4 maxDist[size];
	IsctOptions<f32x4,packetSize,isct::fFullMaxDist|isct::fShadow> options(maxDist,cache[idx]);
	Isct<f32x4,packetSize,AccStruct::isctFlags|isct::fShadow> result; {
		Vec3q lPos(lightPos.x,lightPos.y,lightPos.z);
		RayGroup<packetSize,1,1> rays(&lPos,fromLight,fromLightInv);

		for(int q=0;q<size;q++) maxDist[q]=lightDist[q]*0.99999f;
		result=tree.TraversePacket(rays,sel,options);
	}
	cache[idx]=result.LastShadowTri();

	Vec3q lightColor(light.color);
	for(int q=0;q<size;q++) {
		if(!sel[q]) continue;
		f32x4b mask=lightDist[q]-result.Distance(q)<=lightDist[q]*0.0001f;
		out[q]+=Condition(mask,ShadeLight(lightColor,dot[q],lightDist[q]));
	}

	return result.Stats();
}

/*
Vec3q Sample(const gfxlib::Texture &tex,const Vec2q &uv) {
	Vec2q pos=uv*Vec2q(float(tex.Width()),float(tex.Height()));
	i32x4 x(pos.x),y(pos.y);

	assert(tex.GetFormat().GetIdent()==gfxlib::TI_R8G8B8);
	u8 *data=(u8*)tex.DataPointer();
	int pitch=tex.Pitch();

	for(int n=0;n<4;n++) {
		x[n]=x[n]%tex.Width();
		y[n]=y[n]%tex.Height();
	}
//	x%=i32x4(tex.Width()); y%=i32x4(tex.Height());
	i32x4 offset=x+x+x+y*i32x4(pitch);

	u8 *pix[4]={data+offset[0],data+offset[1],data+offset[2],data+offset[3]};

	Vec3q out;
	out.x[0]=pix[0][0]; out.y[0]=pix[0][1]; out.z[0]=pix[0][2];
	out.x[1]=pix[1][0]; out.y[1]=pix[1][1]; out.z[1]=pix[1][2];
	out.x[2]=pix[2][0]; out.y[2]=pix[2][1]; out.z[2]=pix[2][2];
	out.x[3]=pix[3][0]; out.y[3]=pix[3][1]; out.z[3]=pix[3][2];

	return out*f32x4(1.0f/255.0f);
}*/

//template <class AccStruct,class Group,class Selector>
//void TraceReflection(const AccStruct &tree,TracingContext<Group,Selector> &c);

template <class AccStruct,class Group,class Selector>
void RayTrace(const AccStruct &tree,TracingContext<Group,Selector> &c) {
	enum { primary=0, size=Selector::size };
	
	typedef typename Vec3q::TScalar real;
	typedef typename Vec3q::TBool boolean;
	const floatq maxDist=100000.0f;

	for(int q=0;q<size;q++)
		InitializationShader(c,q,maxDist);

	Isct<f32x4,size,AccStruct::isctFlags> result=
		tree.TraversePacket(c.rays,FullSelector<size>(),IsctOptions<f32x4,size,0>());
	RaySelector<size> &selector=c.selector;
	TreeStats<1> stats=result.Stats();

	for(int q=0;q<size;q++) {
		c.distance[q]=result.Distance(q);
		c.objId[q]=result.Object(q);
		if(AccStruct::isctFlags&isct::fElement) c.elementId[q]=result.Element(q);
	}

	for(int q=0;q<size;q++) {
		f32x4b mask=c.distance[q]<maxDist;
		selector[q]=ForWhich(mask);
		c.color[q]=Vec3q(0.0f,0.0f,0.0f);
		if(!selector[q]) continue;

		i32x4b imask(mask);

		c.position[q]=c.RayDir(q)*c.distance[q]+c.RayOrigin(q);
		
		Vec3f normals[4];
	//	i32x4 objId=c.objId[q][k]&i32x4(imask);
	//	i32x4 elementId=c.elementId[q][k]&i32x4(imask);
		
		for(int k=0;k<4;k++) {
			if(!((i32x4)imask)[k]) continue;
	
			normals[k]=Vec3f(0.0f,1.0f,0.0f);	
			normals[k]=tree.FlatNormals(c.objId[q][k],c.elementId[q][k]);
		}
		Convert(normals,c.normal[q]);
		
	/*	if(c.options.shadingMode==smGouraud)
			c.normal[q]=GouraudNormals(tree.objects,shadingData,Condition(imask,c.objId[q]),c.RayOrigin(q),c.RayDir(q));
		else
			c.normal[q]=FlatNormals<Vec3q>(tree.objects,Condition(imask,c.objId[q]));*/
	
	//	DistanceShader(c,q);
		SimpleLightingShader(c,q);
	//	Vec2q texCoord=GouraudTexCoords(tree.objects,shadingData,Condition(imask,c.objId[q]),c.RayOrigin(q),c.RayDir(q));
	//	c.color[q]*=Sample(tex,texCoord);
	}

	enum { lightsEnabled=0 };

	if(lightsEnabled) {
		Light lights[]={
			Light(Vec3f(0,-1.5f,0),Vec3f(5,5,3)*0.5f),
			Light(Vec3f(1,-1.5f,0),Vec3f(1,5,2)*0.5f),
			Light(Vec3f(0,-1.5f,1),Vec3f(5,1,1)*0.5f), };

		for(int n=0;n<sizeof(lights)/sizeof(Light);n++) {
			stats+=TraceLight<Group::size>(tree,c.distance,c.position,c.normal,c.light,lights[n],
				c.shadowCache,n);
		}
	}

//	if(c.options.reflections>0&&selector.Num())
//		TraceReflection(tree,c);

	if(lightsEnabled) for(int q=0;q<size;q++) {
		c.color[q]=Condition(selector.SSEMask(q),c.color[q]*c.light[q]);
	}
	else for(int q=0;q<size;q++) {
		c.color[q]=Condition(selector.SSEMask(q),c.color[q]);
	}
	if(c.options.rdtscShader)
		for(int q=0;q<Selector::size;q++)
			StatsShader(c,q,stats);
}


/*
template <class AccStruct,class Group,class Selector>
void TraceReflection(const AccStruct &tree,TracingContext<Group,Selector> &c) {
	typedef typename Vec3q::TScalar real;
	typedef typename Vec3q::TBool boolean;

	Vec3q reflDir[Selector::size],idir[Selector::size];

	for(int i=0;i<c.selector.Num();i++) {
		int q=c.selector.Idx(i);
		reflDir[q]=Reflect(c.RayDir(q),c.normal[q]);
		idir[q]=VInv(reflDir[q]);
	}

	typedef RayGroup<Group::size,0,1> TGroup;
	TracingContext<TGroup,Selector> rc(TGroup(c.position,reflDir,idir),c.selector);
	rc.options=c.options;
	rc.options.reflections--;

	RayTrace(tree,rc);
	c.stats.Update(rc.stats);

	for(int i=0;i<c.selector.Num();i++) {
		int q=c.selector.Idx(i);
		c.color[q]=Condition(c.selector.Mask(i),
						c.color[q]*Const<real,8,10>()+rc.color[q]*Const<real,2,10>(),
						c.color[q]);
	}
}
*/


#include "scene.inl"

#endif

