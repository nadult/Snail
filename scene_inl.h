#ifndef RTRACER_SCENE_INL_H
#define RTRACER_SCENE_INL_H

#include "scene.h"

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
			float(stats.GetIntersects())*(0.04f/size),
			float(stats.GetLoopIters())*(0.02f/size),
			float(stats.GetSkips()*0.5f + (stats.GetBreaking()? 0.25 : 0)) );
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


	template <class AccStruct> template <int size,class Selector>
	TreeStats<1> Scene<AccStruct>::TraceLight(const Selector &inputSel,const shading::Sample *samples,
												Vec3q *__restrict__ diffuse,Vec3q *__restrict__ specular,int idx) const {
		enum { shadows=1 };

		TreeStats<1> stats;

		const Light &light=lights[idx];
		Vec3p lightPos=light.pos;

		ALLOCAA(Vec3q, fromLight, size);
		ALLOCAA(Vec3q, idir, size);
		ALLOCAA(floatq, distance, size);
		ALLOCAA(floatq, dot, size);
		ALLOCAA(f32x4b, mask, size);

//		Vec3q fromLight[size], idir[size];
//		floatq distance[size];
//		floatq dot[size];
//		f32x4b mask[size];

		RaySelector<size> sel=inputSel;

		for(int q = 0; q < size; q++) {
			if(!sel[q]) continue;
			const shading::Sample &s=samples[q];

			Vec3q lightVec=(s.position-Vec3q(lightPos));
			f32x4b close=LengthSq(lightVec)<0.0001f;
			lightVec=Condition(close,Vec3q(0.0f,1.0f,0.0f),lightVec);

			distance[q]=Sqrt(lightVec|lightVec);
			fromLight[q]=lightVec*Inv(distance[q]);
			idir[q]=SafeInv(fromLight[q]);
			
			dot[q]=s.normal|fromLight[q];
			mask[q]=dot[q]>0.0f;

			sel[q]&=ForWhich(mask[q]);
		}

		if(shadows) {
			Vec3q lPos(lightPos.x,lightPos.y,lightPos.z);
			Vec3q lColor(light.color.x,light.color.y,light.color.z);
		//	floatq tDistance[size];
			ALLOCAA(floatq, tDistance, size);
			for(int q = 0; q < size; q++)
				tDistance[q] = distance[q]*0.99999f;

			for(int q=0;q<size;q++) stats.TracingRays(CountMaskBits(sel[q]));
			Context<size,isct::fShOrig|isct::fShadow> c(&lPos, fromLight, idir, tDistance, 0, 0, &stats);
			geometry.TraversePacket(c,sel);

			for(int q=0;q<size;q++) {
				if(!sel[q]) continue;

				f32x4 dist=distance[q];
				f32x4b msk=mask[q]&&dist-tDistance[q]<=dist*0.0001f;

				f32x4 atten=dist*light.iRadius;
				atten=Max(f32x4(0.0f), ((floatq(1.0f)-atten) * 0.2f + FastInv(f32x4(16.0f)*atten*atten)) -
						f32x4(0.0625f));

				f32x4 diffMul=dot[q]*atten;
				f32x4 specMul=dot[q]; specMul*=specMul; specMul*=specMul; specMul*=specMul; specMul*=specMul;
				specMul*=atten;
		
				diffuse [q]+=Condition(msk,lColor*diffMul);
				specular[q]+=Condition(msk,lColor*specMul);
			}
		}
		else {
			Vec3q lColor(light.color.x,light.color.y,light.color.z);
			for(int q = 0; q < size; q++) {
				if(!sel[q]) continue;

				f32x4 dist=distance[q];

				f32x4 atten=dist*light.iRadius;
				atten=Max(f32x4(0.0f),((floatq(1.0f)-atten)*0.2f+FastInv(f32x4(16.0f)*atten*atten))-f32x4(0.0625f));

				f32x4 diffMul=dot[q]*atten;
				f32x4 specMul=dot[q];
				specMul*=specMul; specMul*=specMul; specMul*=specMul; specMul*=specMul;
				specMul*=atten;
		
				diffuse [q]+=Condition(mask[q],lColor*diffMul);
				specular[q]+=Condition(mask[q],lColor*specMul);
			}
		}

		return stats;
	}

	template <class AccStruct> template <int size,bool sharedOrigin,class Selector>
	Result<size> Scene<AccStruct>::RayTrace(const RayGroup<size,sharedOrigin> &rays,
													const Selector &inputSelector,Cache &cache) const {
		enum { flags=sharedOrigin?isct::fShOrig|isct::fPrimary:0 };
		Result<size> result;

		enum { primary=0, blockSize=4 };
		static_assert(int(blockSize)==int(shading::blockSize),"Block sizes should be equal");

		const floatq maxDist=1.0f/0.0f;

		floatq tDistance[size];
		i32x4 tObject[size],tElement[size];

		for(int q=0;q<size;q++) tDistance[q]=maxDist;
		for(int q=0;q<size;q++) result.stats.TracingRays(CountMaskBits(inputSelector[q]));

	//	bool doAntialias[size/blockSize]={0,};

		Context<size,flags> tc(rays,tDistance,tObject,tElement,&result.stats);

		geometry.TraversePacket(tc,inputSelector);
		RaySelector<size> selector=inputSelector;
		RaySelector<size> reflSel; reflSel.Clear();
		RaySelector<size> refrSel; refrSel.Clear();

		if(gVals[4]) { //no shading
			for(int q=0;q<size;q++) {
				floatq dist=(Condition(tDistance[q]>maxDist,0.0f,Inv(tDistance[q])));
				floatq dist2=dist*250.0f;
				floatq dist1=dist*20.0f;
				floatq dist3=dist*2.0f;
				result.color[q]=Vec3q(dist1, dist2, dist3);
			}
			return result;
		}
		
		Vec3q minPos( 1.0f/0.0f, 1.0f/0.0f, 1.0f/0.0f);
		Vec3q maxPos(-1.0f/0.0f,-1.0f/0.0f,-1.0f/0.0f);

		ShTriCache &shTriCache=cache.shTriCache;
		//shading::Sample samples[size];
		ALLOCAA(shading::Sample, samples, size);
		int matCount=materials.size();

		for(uint b=0;b<size/blockSize;b++) {
			uint b4=b<<2;

	//		f32x4b mask[blockSize];
	//		i32x4 matId[blockSize],object[blockSize],element[blockSize];
			ALLOCAA(f32x4b, mask, blockSize);
			ALLOCAA(i32x4, matId, blockSize);
			ALLOCAA(i32x4, object, blockSize);
			ALLOCAA(i32x4, element, blockSize);
		
			for(int q=0;q<blockSize;q++) {
				int tq=b4+q;

				shading::Sample &s=samples[tq];

				mask[q]=tDistance[tq]<maxDist&&selector.SSEMask(tq);
				selector[tq]=ForWhich(mask[q]);
				i32x4b imask(mask[q]);

				object[q]=Condition(imask,tObject[tq],i32x4(0));
				element[q]=AccStruct::isctFlags&isct::fElement?Condition(imask,tElement[tq],i32x4(0)):i32x4(0);
				s.position=rays.Dir(tq)*tDistance[tq]+rays.Origin(tq);
				minPos=Condition(mask[q],VMin(minPos,s.position),minPos);
				maxPos=Condition(mask[q],VMax(maxPos,s.position),maxPos);
	
				s.diffuse=s.specular=Vec3q(0.0f,0.0f,0.0f);
				matId[q]=0;
			}

			int mask4=selector.Mask4(b);
			if(!mask4) continue;

			shading::Sample *s=samples+b4;
			int obj0=object[0][0],elem0=element[0][0];

			// 4x4 full, single triangle
			if(mask4==0xf0f0f0f &&
					ForAll( ((object[0]==object[1]&&element[0]==element[1])&&
						 	 (object[2]==object[3]&&element[2]==element[3])) &&
							((object[0]==object[2]&&element[0]==element[2])&&
							 (object[0]==obj0&&element[0]==elem0))  )			) {
				
				uint hash=shTriCache.Hash(obj0,elem0);
				ShTriangle &shTri=shTriCache[hash];
				if(EXPECT_NOT_TAKEN(!shTriCache.SameId(hash,obj0,elem0))) {
					shTri=geometry.GetSElement(obj0,elem0);
					shTriCache.SetId(hash,obj0,elem0);
				}
			
				int tMatId=shTri.matId>=matCount?0:shTri.matId;
				const bool flatNormals=shTri.FlatNormals();
				const bool computeTexCoords=materialFlags[tMatId]&shading::Material::fTexCoords;
				reflSel.Mask4(b)=materialFlags[tMatId]&shading::Material::fReflection?0x0f0f0f0f:0;
				refrSel.Mask4(b)=materialFlags[tMatId]&shading::Material::fRefraction?0x0f0f0f0f:0;

				if(!computeTexCoords) {
					if(flatNormals) {
						Vec3q nrm0(shTri.nrm[0]);
						s[0].normal=s[1].normal=s[2].normal=s[3].normal=nrm0;
					}
					else {
						Vec3p nrm0=shTri.nrm[0];
						Vec3q nrm1(shTri.nrm[1]);
						Vec3q nrm2(shTri.nrm[2]);

						for(int q=0;q<4;q++) {
							Vec2q bar=shTri.Barycentric(rays.Origin(b4+q),rays.Dir(b4+q));
							s[q].normal=Vec3q(nrm0)+(Vec3q(nrm1)*bar.x+Vec3q(nrm2)*bar.y);
						}
					}
				}
				else {
					Vec2q uv0=Vec2q(shTri.uv[0].x,shTri.uv[0].y);
					Vec2q uv1=Vec2q(shTri.uv[1].x,shTri.uv[1].y);
					Vec2q uv2=Vec2q(shTri.uv[2].x,shTri.uv[2].y);
					Vec3p nrm0=shTri.nrm[0];
					Vec3p nrm1=shTri.nrm[1];
					Vec3p nrm2=shTri.nrm[2];

					for(int q=0;q<4;q++) {
						Vec2q bar=shTri.Barycentric(rays.Origin(b4+q),rays.Dir(b4+q));
						s[q].texCoord=uv0+uv1*bar.x+uv2*bar.y;
						Broadcast(Maximize(s[q].texCoord)-Minimize(s[q].texCoord),s[q].texDiff);
						s[q].normal=Vec3q(nrm0)+Vec3q(nrm1)*bar.x+Vec3q(nrm2)*bar.y;
					}
				}
				
				materials[tMatId]->Shade(samples+b4,RayGroup<blockSize,sharedOrigin>(tc.rays,b4),
											cache.samplingCache);
			}
			else {
			//	doAntialias[b]=1;

				for(uint q=0;q<4;q++) {
					uint tq=b4+q;
					if(!selector[tq]) continue;

					shading::Sample &s=samples[tq];

					i32x4b imask(mask[q]);
					int invBitMask=~ForWhich(imask);
					
					int obj0=object[q][0],elem0=element[q][0];
					if(i32x4(imask)[0]) {
						uint hash=shTriCache.Hash(obj0,elem0);
						ShTriangle &shTri=shTriCache[hash];
						if(!shTriCache.SameId(hash,obj0,elem0)) {
							shTri=geometry.GetSElement(obj0,elem0);
							shTriCache.SetId(hash,obj0,elem0);
						}

						int tMatId=shTri.matId>=matCount?0:shTri.matId;
						matId[q]=Condition(imask,i32x4(tMatId+1));

						Vec2q bar=shTri.Barycentric(rays.Origin(tq),rays.Dir(tq));

						s.texCoord=	Vec2q(shTri.uv[0].x,shTri.uv[0].y)+
									Vec2q(shTri.uv[1].x,shTri.uv[1].y)*bar.x+
									Vec2q(shTri.uv[2].x,shTri.uv[2].y)*bar.y;
						Broadcast(Maximize(s.texCoord)-Minimize(s.texCoord),s.texDiff);

						s.normal=Vec3q(shTri.nrm[0])+Vec3q(shTri.nrm[1])*bar.x+Vec3q(shTri.nrm[2])*bar.y;
					}

					if(AccStruct::isctFlags&isct::fElement?ForAny(object[q]!=obj0||element[q]!=elem0):ForAny(object[q]!=obj0))
					  for(int k=1;k<4;k++) {
						int obj=object[q][k],elem=element[q][k];
						if(invBitMask&(1<<k)||(obj==obj0&&elem==elem0)) continue;

						uint hash=shTriCache.Hash(obj,elem);
						ShTriangle &shTri=shTriCache[hash];
						if(!shTriCache.SameId(hash,obj,elem)) {
							shTri=geometry.GetSElement(obj,elem);
							shTriCache.SetId(hash,obj,elem);
						}
					
						int tMatId=shTri.matId>=matCount?0:shTri.matId;
						matId[q][k]=tMatId+1;

						Vec2q bar=shTri.Barycentric(rays.Origin(tq),rays.Dir(tq));

						Vec2q tex=Vec2q(shTri.uv[0].x,shTri.uv[0].y)+
								  Vec2q(shTri.uv[1].x,shTri.uv[1].y)*bar.x+
								  Vec2q(shTri.uv[2].x,shTri.uv[2].y)*bar.y;

						Vec2f diff=Maximize(tex)-Minimize(tex);
						s.texDiff.x[k]=diff.x; s.texDiff.y[k]=diff.y;
						s.texCoord.x[k]=tex.x[k]; s.texCoord.y[k]=tex.y[k];
						Vec3f nrm=shTri.nrm[0]+shTri.nrm[1]*bar.x[k]+shTri.nrm[2]*bar.y[k];
						
						s.normal.x[k]=nrm.x; s.normal.y[k]=nrm.y; s.normal.z[k]=nrm.z;
					  }
				}
				int matId0=matId[0][0];

				if(ForAll(	((matId[0]==matId[1])&&(matId[2]==matId[3]))&&
							((matId[0]==matId[2])&&(matId[1]==matId0     )) )) {
					reflSel.Mask4(b)=materialFlags[matId0-1]&shading::Material::fReflection?0x0f0f0f0f:0;
					refrSel.Mask4(b)=materialFlags[matId0-1]&shading::Material::fRefraction?0x0f0f0f0f:0;

					if(EXPECT_TAKEN(matId0))
						materials[matId0-1]->Shade
							(samples+b4,RayGroup<blockSize,sharedOrigin>(tc.rays,b4),cache.samplingCache);
				}
				else {	
					static_assert(blockSize==4,"If you want some other value, you will have to modify the code here");

					i32x4 matIds[blockSize]={i32x4(0),i32x4(0),i32x4(0),i32x4(0)};
					uint matIdCount=0;
					f32x4b mask[4];

					for(int q=0;q<blockSize;q++) {
						const i32x4 &tMatId=matId[q];

						u8 reflMask=0,refrMask=0;
						for(int k=0;k<4;k++) {
							int id=tMatId[k];
							i32x4 kMatId(id);
							reflMask|=id&&materialFlags[id-1]&shading::Material::fReflection?1<<k:0;
							refrMask|=id&&materialFlags[id-1]&shading::Material::fRefraction?1<<k:0;

							if(ForAll((matIds[0]!=kMatId&&matIds[1]!=kMatId)&&(matIds[2]!=kMatId&&matIds[3]!=kMatId))) {
								matIds[matIdCount>>2][matIdCount&3]=id;
								matIdCount++;
							}
						}
						reflSel[q+b4]=reflMask;
						refrSel[q+b4]=refrMask;
					}

					for(uint n=0;n<matIdCount;n++) {
						uint id=matIds[n>>2][n&3];
						i32x4 id4(id);
						mask[0]=matId[0]==id4;
						mask[1]=matId[1]==id4;
						mask[2]=matId[2]==id4;
						mask[3]=matId[3]==id4;

						materials[id-1]->Shade
							(samples+b4,mask,RayGroup<blockSize,sharedOrigin>(tc.rays,b4),cache.samplingCache);
					}
				}
			}

		}


		if(reflSel.Any() && !gVals[5] && cache.reflections < 1) {
			cache.reflections++;
			Result<size> reflResult = TraceReflection(rays.DirPtr(),samples,reflSel,cache);
			result.stats += reflResult.stats;
			cache.reflections--;

			for(int q=0;q<size;q++) {
				samples[q].diffuse=Condition(reflSel.SSEMask(q),
								samples[q].diffuse+(reflResult.color[q]-samples[q].diffuse)*floatq(0.3f),
								samples[q].diffuse );
			}
		}
		if(refrSel.Any() && !gVals[5] && (flags&isct::fPrimary)) {
			Result<size> refrResult = TraceRefraction(rays,samples,refrSel,cache);
			result.stats += refrResult.stats;

			for(int q=0;q<size;q++) {
				samples[q].diffuse=Condition(refrSel.SSEMask(q),
								samples[q].diffuse+(refrResult.color[q]-samples[q].diffuse)*floatq(0.3f),
								samples[q].diffuse );
			}
		}

	//	Vec3q lDiffuse[size],lSpecular[size];
		ALLOCAA(Vec3q, lDiffuse, size);
		ALLOCAA(Vec3q, lSpecular, size);
		if(lights.size()) {
			Vec3q dif(ambientLight),spec(0.0f,0.0f,0.0f);
			for(int q=0;q<size;q++) {
				lDiffuse[q] = dif;
				lSpecular[q] = spec;
			}
		}
		{
			int nLights=lights.size();
			Vec3f tMinPos(Minimize(minPos)),tMaxPos(Maximize(maxPos));
			for(int n=0;n<nLights;n++) {
				const Light &light=lights[n];
				if(BoxPointDistanceSq(BBox(tMinPos,tMaxPos),light.pos)>light.radSq)
					continue;

				result.stats+=TraceLight<size>(selector,samples,lDiffuse,lSpecular,n);
			}
		}

		if(lights.size()) {
			Vec3q zero(0.0f,0.0f,0.0f),one(1.0f,1.0f,1.0f);
			for(int q=0;q<size;q++) {
				result.color[q]=samples[q].diffuse*lDiffuse[q]+samples[q].specular*lSpecular[q];
			}
		}
		else for(int q=0;q<size;q++)
			result.color[q]=samples[q].diffuse;

		if(gVals[2] && flags&isct::fPrimary) {
			Vec3q col=StatsShader<size>(result.stats)[0];
			for(int q=0;q<size;q++)
				result.color[q]=col;
		}

	/*	if(isct::fPrimary&&sharedOrigin&&!cache.supersampling&&!gVals[6]) {
			cache.supersampling=1;

			for(uint b=0;b<size/blockSize;b++) {
				uint b4=b<<2;

				if(!doAntialias[b]) continue;

				Vec3q dir[blockSize*4],idir[blockSize*4];
				for(int q=0;q<blockSize;q++) {
					Vec3f dirs[4]; Convert(rays.Dir(q+b4),dirs);
					dir[q*4+0]=rays.Dir(q+b4)+(Vec3q)((dirs[1]-dirs[0])*0.33333f);
					dir[q*4+1]=rays.Dir(q+b4)-(Vec3q)((dirs[1]-dirs[0])*0.33333f);
					dir[q*4+2]=rays.Dir(q+b4)+(Vec3q)((dirs[2]-dirs[0])*0.33333f);
					dir[q*4+3]=rays.Dir(q+b4)-(Vec3q)((dirs[2]-dirs[0])*0.33333f);
				}
				for(int q=0;q<blockSize*4;q++) idir[q]=VInv(dir[q]);

				Result<blockSize*4> sup=RayTrace(RayGroup<blockSize*4,1>(rays.OriginPtr(),dir,idir),
													FullSelector<blockSize*4>(),cache);

				for(int q=0;q<blockSize;q++)
					result.color[b4+q]=(result.color[b4+q ]+
						sup.color[q*4+0]+sup.color[q*4+1]+sup.color[q*4+2]+sup.color[q*4+3] )*f32x4(1.0f/5.0f);
				result.stats += sup.stats;
			}
				
			cache.supersampling=0;
		} */

		return result;
	}

	template <class AccStruct> template <int size,template <int> class Selector>
	Result<size> Scene<AccStruct>::TraceReflection(const Vec3q *__restrict__ dir,
				const shading::Sample *__restrict__ samples,const Selector<size> &selector,Cache &cache) const {
		Result<size> result;
		Vec3q reflDir[size],reflOrig[size],idir[size];

		for(int q=0;q<size;q++) {
			reflDir[q]=Reflect(dir[q],samples[q].normal);
			reflOrig[q]=samples[q].position+reflDir[q]*floatq(0.001f);
			idir[q]=SafeInv(reflDir[q]);
		}
		
		result=RayTrace(RayGroup<size,0>(reflOrig,reflDir,idir),selector,cache);
		return result;
	}

	template <class AccStruct> template <int size,bool sharedOrigin,template <int> class Selector>
	Result<size> Scene<AccStruct>::TraceRefraction(const RayGroup<size,sharedOrigin> &rays,
				const shading::Sample *__restrict__ samples,const Selector<size> &selector,Cache &cache) const {
		Result<size> result;
		Vec3q refrOrig[size];

		for(int q=0;q<size;q++)
			refrOrig[q]=samples[q].position+rays.Dir(q)*floatq(0.001f);
		
		result=RayTrace(RayGroup<size,0>(refrOrig,rays.DirPtr(),rays.IDirPtr()),selector,cache);
		return result;
	}

#endif

