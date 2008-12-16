#ifndef RTRACER_SCENE_INL_H
#define RTRACER_SCENE_INL_H

#include "scene.h"

	template <class AccStruct> template <int size,class Selector>
	TreeStats<1> Scene<AccStruct>::TraceLight(const Selector &inputSel,const shading::Sample *samples,
												Vec3q *__restrict__ diffuse,Vec3q *__restrict__ specular,int idx) const {
		enum { shadows=1 };

		const Light &light=lights[idx];
		Vec3p lightPos=light.pos;
		Vec3q fromLight[size];
		floatq distance[size];
		floatq dot[size];
		f32x4b mask[size];

		RaySelector<size> sel=inputSel;

		for(int q=0;q<size;q++) {
			if(!sel[q]) continue;
			const shading::Sample &s=samples[q];

			Vec3q lightVec=(s.position-Vec3q(lightPos));
			f32x4b close=LengthSq(lightVec)<0.0001f;
			lightVec=Condition(close,Vec3q(0.0f,1.0f,0.0f),lightVec);

			distance[q]=Sqrt(lightVec|lightVec);
			fromLight[q]=lightVec*Inv(distance[q]);
			
			dot[q]=s.normal|fromLight[q];
			mask[q]=dot[q]>0.0f;

			sel[q]&=ForWhich(mask[q]);
		}

		Isct<f32x4,size,AccStruct::isctFlags|isct::fShadow|isct::fMaxDist> hit;

		if(shadows) {
			Vec3q lPos(lightPos.x,lightPos.y,lightPos.z);
			Vec3q lColor(light.color.x,light.color.y,light.color.z);

			RayGroup<size,isct::fShOrig|isct::fInvDir|isct::fMaxDist|isct::fShadow> rays(&lPos,fromLight);

			for(int q=0;q<size;q++) rays.maxDist.Set(q,distance[q]*0.99999f);
			hit=geometry.TraversePacket(rays,sel);

			for(int q=0;q<size;q++) {
				if(!sel[q]) continue;

				f32x4 dist=distance[q];
				f32x4b msk=mask[q]&&dist-hit.Distance(q)<=dist*0.0001f;

				f32x4 iDistSq=Inv(dist*dist);
				f32x4 diffMul=dot[q]*iDistSq;
				f32x4 specMul=dot[q]; specMul*=specMul; specMul*=specMul; specMul*=specMul; specMul*=specMul;
				specMul*=iDistSq;
		
				diffuse [q]+=Condition(msk,lColor*diffMul);
				specular[q]+=Condition(msk,lColor*specMul);
			}
		}
		else {
			Vec3q lColor(light.color.x,light.color.y,light.color.z);
			for(int q=0;q<size;q++) {
				if(!sel[q]) continue;

				f32x4 dist=distance[q];

				f32x4 iDistSq=Inv(dist*dist);
				f32x4 diffMul=dot[q]*iDistSq;
				f32x4 specMul=dot[q]; specMul*=specMul; specMul*=specMul;
				specMul*=iDistSq;
		
				diffuse [q]+=Condition(mask[q],lColor*diffMul);
				specular[q]+=Condition(mask[q],lColor*specMul);
			}
		}

		return hit.Stats();
	}

	template <class AccStruct> template <int flags,int size,class Selector>
	Result<size> Scene<AccStruct>::RayTrace(const RayGroup<size,flags> &rays,
													const Selector &inputSelector,Cache &cache) const {
		enum { primary=0, blockSize=4 };
		static_assert(int(blockSize)==int(shading::BaseMaterial::blockSize),"Block sizes should be equal");

		const floatq maxDist=100000.0f;

		Isct<f32x4,size,AccStruct::isctFlags> hit=geometry.TraversePacket(rays,inputSelector);
		RaySelector<size> selector=inputSelector;

		if(gVals[4]) { //no shading
			Result<size> result;
			result.stats=hit.Stats();
			for(int q=0;q<size;q++) {
				floatq dist=(Condition(hit.Distance(q)>maxDist,0.0f,Inv(hit.Distance(q))));
				floatq dist2=dist*250.0f;
				floatq dist1=dist*20.0f;
				floatq dist3=dist*2.0f;
				result.color[q]=Vec3q(dist1,dist2,dist3);
			}
			return result;
		}

		ShTriCache &shTriCache=cache.shTriCache;
		shading::Sample samples[size];
		int matCount=materials.size();

		for(uint b=0;b<size/blockSize;b++) {
			uint b4=b<<2;

			f32x4b mask[blockSize];
			i32x4 matId[blockSize],object[blockSize],element[blockSize];

			for(int q=0;q<blockSize;q++) {
				int tq=b4+q;

				shading::Sample &s=samples[tq];

				mask[q]=hit.Distance(tq)<maxDist&&selector.SSEMask(tq);
				selector[tq]=ForWhich(mask[q]);
				i32x4b imask(mask[q]);

				object[q]=Condition(imask,hit.Object(tq),i32x4(0));
				element[q]=AccStruct::isctFlags&isct::fElement?Condition(imask,hit.Element(tq),i32x4(0)):i32x4(0);
				s.position=rays.Dir(tq)*hit.Distance(tq)+rays.Origin(tq);
	
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
				if(__builtin_expect(!shTriCache.SameId(hash,obj0,elem0),0)) {
					shTri=geometry.GetSElement(obj0,elem0);
					shTriCache.SetId(hash,obj0,elem0);
				}
			
				int tMatId=shTri.matId>=matCount?0:shTri.matId;
				const bool flatNormals=shTri.FlatNormals();
				const bool computeTexCoords=materialFlags[tMatId]&shading::BaseMaterial::fTexCoords;

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
				
				materials[tMatId]->Shade(samples+b4,PRayGroup<size,flags>(rays,b4),cache.samplingCache);
			}
			else {
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
					if(EXPECT_TAKEN(matId0))
						materials[matId0-1]->Shade(samples+b4,PRayGroup<size,flags>(rays,b4),cache.samplingCache);
				}
				else {	
					static_assert(blockSize==4,"If you want some other value, you will have to modify the code here");

					i32x4 matIds[blockSize]={i32x4(0),i32x4(0),i32x4(0),i32x4(0)};
					uint matIdCount=0;
					f32x4b mask[4];

					for(int q=0;q<blockSize;q++) {
						const i32x4 &tMatId=matId[q];

						for(int k=0;k<4;k++) {
							int id=tMatId[k];
							i32x4 kMatId(id);
							if(ForAll((matIds[0]!=kMatId&&matIds[1]!=kMatId)&&(matIds[2]!=kMatId&&matIds[3]!=kMatId))) {
								matIds[matIdCount/4][matIdCount&3]=id;
								matIdCount++;
							}
						}
					}

					for(int n=0;n<matIdCount;n++) {
						int id=matIds[n/4][n&3];
						i32x4 id4(id);
						mask[0]=matId[0]==id4;
						mask[1]=matId[1]==id4;
						mask[2]=matId[2]==id4;
						mask[3]=matId[3]==id4;

						materials[id-1]->Shade(samples+b4,mask,PRayGroup<size,flags>(rays,b4),cache.samplingCache);
					}
				}
			}
		}

		Result<size> result;
		result.stats=hit.Stats();

		Vec3q lDiffuse[size],lSpecular[size];
		if(lights.size()) {
			Vec3q dif(ambientLight),spec(0.0f,0.0f,0.0f);
			for(int q=0;q<size;q++) {
				lDiffuse[q]=dif;
				lSpecular[q]=spec;
			}
		}
		int nLights=Min(int(shading::Sample::maxLightSamples),lights.size());

		for(int n=0;n<nLights;n++)
			result.stats+=TraceLight<size>(selector,samples,lDiffuse,lSpecular,n);

		if(lights.size())
			for(int q=0;q<size;q++)
				result.color[q]=samples[q].diffuse*lDiffuse[q]+samples[q].specular*lSpecular[q];
		else
			for(int q=0;q<size;q++)
				result.color[q]=samples[q].diffuse;

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

	template <class AccStruct> template <int flags,int size,class Selector>
	Result<size> Scene<AccStruct>::TraceReflection(const RayGroup<size,flags> &rays,const Selector &selector,
								const Vec3q *position,const Vec3q *normal) const {
		Vec3q reflDir[size];

		for(int q=0;q<size;q++)
			reflDir[q]=Reflect(rays.Dir(q),normal[q]);
		RayGroup<size,isct::fInvDir> tRays(position,reflDir);

		return RayTrace(tRays,selector);
	}

#endif

