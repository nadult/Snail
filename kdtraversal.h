#ifndef KDTRAVERSAL_H
#define KDTRAVERSAL_H

template <class Output,class Vec,class base>
void KDTree::FullTraverse(const Vec &rOrigin,const Vec &rDir,const Output &out) const {	
	if(Output::objectIdsFlag) {
		for(int m=0;m<ScalarInfo<base>::multiplicity;m++)
			out.object[m]=0;
	}

	TreeStats stats;
	stats.TracingRay();

	for(int n=0;n<objects.size();n++) {
		stats.Intersection();

		const Object &obj=objects[n];
		base dst=obj.Collide(rOrigin,rDir);
		typename Vec::TBool mask=dst>Const<base,0>()&&dst<out.dist[0];

		if(Output::objectIdsFlag) {
			i32x4b test=mask;
			out.object[0]=Condition(test,i32x4(n),out.object[0]);
		}
		out.dist[0]=Condition(mask,dst,out);
	}

	out.stats->Update(stats);
}

template <class Output>
INLINE void KDTree::TraverseMono(const Vec3p &rOrigin,const Vec3p &tDir,const Output &out) const {
	struct Locals4 {
		float tMin,tMax;
		const KDNode *node;
	};
	Locals4 stackBegin[MaxLevel+16],*stack=stackBegin;

	Vec3p rDir=Vec3p(tDir.x+0.000000000001f,tDir.y+0.000000000001f,tDir.z+0.000000000001f);
	Vec3p invDir=VInv(rDir);

	const float *rStart=(const float*)&rOrigin;
	const float *irDir=(const float*)&invDir;
	float tMin=ConstEpsilon<float>(),tMax=out.dist[0];

	if(Output::objectIndexes)
		out.object[0]=0;

	int signMask=SignMask(floatq(invDir.m));
	int dSign[3]={signMask&1?0:1,signMask&2?0:1,signMask&4?0:1};

	const KDNode *node=&nodes[0];
	const u32 *objIds=&objectIds[0];
	const Object *objs=&objects[0];

	TreeStats stats;
	stats.TracingRay();

	while(true) {
		stats.LoopIteration();
		u32 axis=node->Axis();

		if(axis==3) { // leaf
			if(node->NumObjects()) {
				const u32 *id=objIds+node->FirstObject();

				float eps=0.0001f,ttMin=tMin-eps,ttMax=tMax+eps;
				float minRet=out.dist[0];

				for(int n=node->NumObjects();n>0;n--) {
					stats.Intersection();

					int tid=*id++;
					const Object &obj=objs[tid];
					float ret; Convert(obj.Collide(rOrigin,rDir),ret);
			
					if(ret>ttMin&&ret<ttMax) {
						if(Output::objectIndexes)
							out.object[0]=tid;
						minRet=Min(minRet,ret);
					}
				}

				if(minRet<out.dist[0]) {
					out.stats->Update(stats);
					out.dist[0]=minRet;
					return;
				}
			}

POPSTACK:
			if(stack==stackBegin) {
				out.stats->Update(stats);
				return;
			}
			{
				stack--;
				tMin=stack->tMin;
				tMax=stack->tMax;
				node=stack->node;
			}
			continue;
		}

		float tSplit=float(node->Pos()-rStart[axis])*irDir[axis];
		node+=node->ChildDist();
		u32 sign=dSign[axis];

		if(ForAll(tSplit<=tMin)) {
			tMin=Max(tSplit,tMin); node+=sign;
			continue;
		}
		if(ForAll(tSplit>=tMax)) {
			tMax=Min(tSplit,tMax); node+=!sign;
			continue;
		}

		stack->tMin=Max(tSplit,tMin);
		stack->tMax=tMax;
		stack->node=node+sign;
		stack++;

		tMax=Min(tMax,tSplit); node+=!sign;
	}
}


/*!
	(Absolute)
	px = [ dy[0].y/dy[0].x	dz[0].z/dz[0].x		dy[1].y/dy[1].x		dz[1].z/dz[1].x	]
	py = [ dx[0].x/dx[0].y	dz[0].z/dz[0].y		dx[1].x/dx[1].y		dz[1].z/dz[1].y	]
	pz = [ dx[0].x/dx[0].z	dy[0].y/dy[0].z		dx[1].x/dx[1].z		dy[1].y/dy[1].z	]
*/
template <int size>
INLINE void ComputePV(const RaySelector<size> &sel,const Vec3q *rDir,floatq *pv)
{
	//	ab[k] = k==0? min(a/b)  :   max(a/b)
	floatq yx[2],zx[2],xy[2],zy[2],xz[2],yz[2];

	{
		const Vec3q dir=VAbs(rDir[sel[0]]);

		floatq ix=Inv(dir.x),iy=Inv(dir.y),iz=Inv(dir.z);
		yx[0]=yx[1]=dir.y*ix;
		zx[0]=zx[1]=dir.z*ix;
		xy[0]=xy[1]=dir.x*iy;
		zy[0]=zy[1]=dir.z*iy;
		xz[0]=xz[1]=dir.x*iz;
		yz[0]=yz[1]=dir.y*iz;
	}

	for(int id=1;id<sel.Num();id++) {
		const Vec3q dir=VAbs(rDir[sel[id]]);

		floatq t1,t2,i;
		i=Inv(dir.x); t1=dir.y*i; t2=dir.z*i;
		yx[0]=Min(t1,yx[0]); yx[1]=Max(t1,yx[1]);
		zx[0]=Min(t2,zx[0]); zx[1]=Max(t2,zx[1]);

		i=Inv(dir.y); t1=dir.x*i; t2=dir.z*i;
		xy[0]=Min(t1,xy[0]); xy[1]=Max(t1,xy[1]);
		zy[0]=Min(t2,zy[0]); zy[1]=Max(t2,zy[1]);

		i=Inv(dir.z); t1=dir.x*i; t2=dir.y*i;
		xz[0]=Min(t1,xz[0]); xz[1]=Max(t1,xz[1]);
		yz[0]=Min(t2,yz[0]); yz[1]=Max(t2,yz[1]);
	}

	__m128 tp1,tp2;

	tp1=_mm_min_ps(_mm_unpacklo_ps(yx[0].m,zx[0].m),_mm_unpackhi_ps(yx[0].m,zx[0].m));
	tp2=_mm_max_ps(_mm_unpacklo_ps(yx[1].m,zx[1].m),_mm_unpackhi_ps(yx[1].m,zx[1].m));
	tp1=_mm_min_ps(tp1,_mm_shuffle(2+3*4+0*16+1*64,tp1));
	tp2=_mm_max_ps(tp2,_mm_shuffle(2+3*4+0*16+1*64,tp2));
	pv[0].m=_mm_shuffle_ps(tp1,tp2,0+1*4+0*16+1*64);

	tp1=_mm_min_ps(_mm_unpacklo_ps(xy[0].m,zy[0].m),_mm_unpackhi_ps(xy[0].m,zy[0].m));
	tp2=_mm_max_ps(_mm_unpacklo_ps(xy[1].m,zy[1].m),_mm_unpackhi_ps(xy[1].m,zy[1].m));
	tp1=_mm_min_ps(tp1,_mm_shuffle(2+3*4+0*16+1*64,tp1));
	tp2=_mm_max_ps(tp2,_mm_shuffle(2+3*4+0*16+1*64,tp2));
	pv[1].m=_mm_shuffle_ps(tp1,tp2,0+1*4+0*16+1*64);

	tp1=_mm_min_ps(_mm_unpacklo_ps(xz[0].m,yz[0].m),_mm_unpackhi_ps(xz[0].m,yz[0].m));
	tp2=_mm_max_ps(_mm_unpacklo_ps(xz[1].m,yz[1].m),_mm_unpackhi_ps(xz[1].m,yz[1].m));
	tp1=_mm_min_ps(tp1,_mm_shuffle(2+3*4+0*16+1*64,tp1));
	tp2=_mm_max_ps(tp2,_mm_shuffle(2+3*4+0*16+1*64,tp2));
	pv[2].m=_mm_shuffle_ps(tp1,tp2,0+1*4+0*16+1*64);
}

template <class Group,int size>
INLINE void ComputeMinMaxOrigin(const RaySelector<size> &sel,const Group &group,Vec3p &min,Vec3p &max) {
	if(Group::singleOrigin) {
		Vec3p pOR[4]; Convert(group.Origin(sel[0]),pOR);
		min=max=pOR[0];
	}
	else {
		Vec3p pOR[4]; Convert(group.Origin(sel[0]),pOR);
		Vec3p tMin=VMin(VMin(pOR[0],pOR[1]),VMin(pOR[2],pOR[3]));
		Vec3p tMax=VMax(VMax(pOR[0],pOR[1]),VMax(pOR[2],pOR[3]));
		for(int i=1;i<sel.Num();i++) {
			int q=sel[i];
			Convert(group.Origin(q),pOR);
			tMin=VMin( VMin(VMin(pOR[0],pOR[1]),VMin(pOR[2],pOR[3])), tMin);
			tMax=VMax( VMax(VMax(pOR[0],pOR[1]),VMax(pOR[2],pOR[3])), tMax);
		}

		min=tMin;
		max=tMax;
	}
}

/*!
	ox = [ minOR.y-maxOr.x*px[0]	minOR.z-maxOr.x*px[1]	maxOR.y-minOr.x*px[2]	maxOR.z-minOr.x*px[3] ]
	oy = [ minOR.x-maxOr.y*py[0]	minOR.z-maxOr.y*py[1]	maxOR.x-minOr.y*py[2]	maxOR.z-minOr.y*py[3] ]
	oz = [ minOR.x-maxOr.z*pz[0]	minOR.y-maxOr.z*pz[1]	maxOR.x-minOr.z*pz[2]	maxOR.y-minOr.z*pz[3] ]
*/
INLINE void ComputeOV(const Vec3p &min,const Vec3p &max,const floatq *pv,floatq *ov) {
	ov[0].m=_mm_sub_ps(
		_mm_shuffle_ps(_mm_shuffle(1+(2<<2),min.m),_mm_shuffle(1+(2<<2),max.m),0+(1<<2)+(0<<4)+(1<<6)), // yzyz
		_mm_mul_ps(_mm_shuffle_ps(max.m,min.m,0+0*4+0*16+0*64),pv[0].m)); //xxxx
	ov[1].m=_mm_sub_ps(
		_mm_shuffle_ps(_mm_shuffle(0+(2<<2),min.m),_mm_shuffle(0+(2<<2),max.m),0+(1<<2)+(0<<4)+(1<<6)), // xzxz
		_mm_mul_ps(_mm_shuffle_ps(max.m,min.m,1+1*4+1*16+1*64),pv[1].m)); //yyyy
	ov[2].m=_mm_sub_ps(
		_mm_shuffle_ps(_mm_shuffle(0+(1<<2),min.m),_mm_shuffle(0+(1<<2),max.m),0+(1<<2)+(0<<4)+(1<<6)), // xyxy
		_mm_mul_ps(_mm_shuffle_ps(max.m,min.m,2+2*4+2*16+2*64),pv[2].m)); //zzzz
}


template <class Output,class Group>
inline void KDTree::TraverseFast(Group &group,const RaySelector<Group::size> &tSelector,const floatq &maxD,const Output &out) const
{
	RaySelector<Group::size> sel=tSelector;

	int xSign=sel.SignMaskX(),ySign=sel.SignMaskY(),zSign=sel.SignMaskZ();
	int sign0[3]={xSign==0,ySign==0,zSign==0};
	
	Vec3p negMask; int negMaskI[3]; {
		float negMaskXYZ[4];
		negMaskXYZ[0]=xSign?-1.0f:1.0f;
		negMaskXYZ[1]=ySign?-1.0f:1.0f;
		negMaskXYZ[2]=zSign?-1.0f:1.0f;
		negMaskXYZ[3]=0.0f;
		negMaskI[0]=xSign?0x80000000:0;
		negMaskI[1]=ySign?0x80000000:0;
		negMaskI[2]=zSign?0x80000000:0;
		Convert(negMaskXYZ,negMask);
	}

	for(int i=0;i<sel.Num();i++) {
		int q=sel[i];

		out.dist[q]=maxD;
		if(Output::objectIndexes)
			out.object[q]=intq(0);
	}

	// Minimized / maximized ray origin
	Vec3p minOR,maxOR; {
		ComputeMinMaxOrigin(sel,group,minOR,maxOR);
		minOR*=negMask; maxOR*=negMask;
		Vec3p tMin=VMin(minOR,maxOR);
		maxOR=VMax(minOR,maxOR); minOR=tMin;	
	}

	floatq pv[3],ov[3];
	ComputePV(sel,&group.Dir(0),pv);
	ComputeOV(minOR,maxOR,pv,ov);

	TreeStats stats;
	stats.TracingPacket(sel.Num()*4);

	Vec3p boxStack[(MaxLevel+8)*2];
	const KDNode *nodeStack[MaxLevel+8];
	u32 stackPos=0,axis;

	char active[Group::size];
	for(int i=0;i<sel.Num();i++)
		active[i]=15;

	const KDNode *node=&nodes[0];
	const u32 *objIds=&objectIds[0];
	const Object *objs=&objects[0];

	Vec3p bMin,bMax; {
		Vec3p tMin=pMin*negMask,tMax=pMax*negMask;
		Vec3p tpMin=VMin(tMin,tMax),tpMax=VMax(tMin,tMax);
		bMin=VMax(minOR,tpMin);
		bMax=tpMax;
	}

	Vec3p beamDir,beamOrig; float beamM,beamA;
	if(!Output::shadow) if(sel.Num()>2) {
		beamDir=Vec3p(0,0,0);
		Vec3q tmp=group.Dir(sel[0]);
		for(int n=1;n<sel.Num();n++) tmp+=group.Dir(sel[n]);
		Vec3p t[4]; Convert(tmp,t);
		beamDir=t[0]+t[1]+t[2]+t[3];
		beamDir*=RSqrt(beamDir|beamDir);
		beamOrig=(minOR+maxOR)*0.5f*negMask;
		Convert(Sqrt((maxOR-minOR)|(maxOR-minOR)),beamA);
		beamA*=0.5f;

		floatq minDot=Const<floatq,1>();
		Vec3q maxVec=group.Dir(sel[0]),mid=Vec3q(beamDir);

		for(int n=0;n<sel.Num();n++) {
			Vec3q &vec=group.Dir(sel[n]);
			floatq dot=vec|mid;
			typename Vec3q::TBool mask=dot<minDot;
			maxVec=Condition(mask,vec,maxVec);
			minDot=Min(dot,minDot);
		}

		floatq maxDist=Length(maxVec-mid*minDot);

		{ float t[4]; Convert(maxDist,t); beamM=Max(Max(t[0],t[1]),Max(t[2],t[3])); }
	}
	ObjectIdxBuffer<8> idxBuffer;
	float maxNodeDistSq,srcSize; {
		Vec3p tmp=maxOR-minOR;
		srcSize=Sqrt(tmp|tmp);
		maxNodeDistSq=maxD[0]+srcSize;
		maxNodeDistSq*=maxNodeDistSq;
	}

	while(true) {
		stats.LoopIteration();

		axis=node->Axis();
		if(axis==3) { // Leaf
			if(node->NumObjects()) {

				const u32 *oid=objIds+node->FirstObject();
				
				Vec3p nodeMin,nodeMax; {
					Vec3p nodeEps(Const<floatq,1,1000>().m);
					Vec3p tMin=bMin*negMask,tMax=bMax*negMask;
					nodeMin=VMin(tMin,tMax)-nodeEps;
					nodeMax=VMax(tMin,tMax)+nodeEps;
				}

				for(int n=node->NumObjects();n>0;n--) {
					int tid=*oid++;
					const Object &obj=objs[tid];
					const bool fullInNode=0;//obj.FullInNode();
					
					if(idxBuffer.Find(tid)) { stats.Skip(); continue; }

					int beamCollision=1;
					if(!Output::shadow) if(sel.Num()>2) {
						beamCollision=obj.BeamCollide(beamOrig,beamDir,beamM,beamA);

					 	if(!beamCollision) { idxBuffer.Insert(tid); continue; }

						// Visualizing beam collisions
						/*	for(int i=0;i<sel.Num();i++) {
							int q=sel[i];
							Vec3q tmp(bMin.x,bMin.y,bMin.z);
							out.dist[q]=Length(group.Origin(q)-tmp);
							out.object[q]=i32x4(0);
						}
						out.stats->Update(stats);
						return; */
					}

				//	if(beamCollision==2) stats.NotBreaking();
				//	else stats.Breaking();

					bool fullInside=1;

					bool allCollided=1;
					floatq newMaxD=Const<floatq,0>();

					for(int i=0;i<sel.Num();i++) {
						int q=sel[i];
						stats.Intersection();

						floatq ret=obj.Collide(group.Origin(q),group.Dir(q));
						f32x4b mask=(ret>Const<floatq,0>()&&ret<out.dist[q]);
						u32 msk=ForWhich(mask);
						if(msk!=15) allCollided=0;

						if(!fullInNode) {
							Vec3q col=group.Origin(q)+group.Dir(q)*ret;
							f32x4b	 insideMask=	col.x>=nodeMin.x&&col.x<=nodeMax.x&&
													col.y>=nodeMin.y&&col.y<=nodeMax.y&&
													col.z>=nodeMin.z&&col.z<=nodeMax.z;
							u32 insideMsk=ForWhich(insideMask);
							mask=mask&&insideMask; 
							// Jakies promienie uderzyly w obiekt poza KD-nodem
							if(msk^insideMsk) fullInside=0;
							msk&=insideMsk;
						}

						if(Output::objectIndexes) {
							i32x4b test=mask;
							i32x4 &dst=out.object[q];
							dst=Condition(test,i32x4(tid),dst);
						}

						newMaxD=Max(newMaxD,ret);

						active[i]^=msk;
						out.dist[q]=Condition(mask,ret,out.dist[q]);
					}

					if(allCollided) {
						float newMax=Max(Max(newMaxD[0],newMaxD[1]),Max(newMaxD[2],newMaxD[3]))+srcSize;
						maxNodeDistSq=Min(maxNodeDistSq,newMax*newMax);
					}
					
					if(fullInside) idxBuffer.Insert(tid);
				}

				for(int i=0;i<sel.Num();i++) if(!active[i]) {
					active[i]=active[sel.Num()-1];
					sel.Disable(i--);
				}
				if(sel.Num()==0) { out.stats->Update(stats); return; }
			}

POPSTACK:
			if(stackPos==0) { out.stats->Update(stats); return; }

			stackPos--;
			bMin=boxStack[stackPos*2+0]; {
				Vec3p tmp=minOR-bMin;
				if((tmp|tmp)>maxNodeDistSq) goto POPSTACK;
			}

			bMax=boxStack[stackPos*2+1];
			node=nodeStack[stackPos];

			continue;
		}

		f32x4 split; {
			union { float f; int i; };
			f=node->Pos();
			i^=negMaskI[axis];
			split=f;
		}
		node+=node->ChildDist();

		u32 mask=1<<axis;
		if(_mm_movemask_ps(_mm_cmpgt_ps(bMin.m,split.m))&mask) { node+=sign0[axis]; continue; }
		if(_mm_movemask_ps(_mm_cmplt_ps(bMax.m,split.m))&mask) { node+=sign0[axis]^1; continue; }

		{
			f32x4 abab=pv[axis]*split+ov[axis],tmp,tt[3];
			tmp=_mm_unpacklo_ps(split.m,abab.m);
			tt[0].m=_mm_shuffle(0+(1<<2)+(3<<4),tmp.m);
			tt[1].m=_mm_shuffle(1+(0<<2)+(3<<4),tmp.m);
			tt[2].m=_mm_shuffle(1+(3<<2)+(0<<4),tmp.m);
			boxStack[stackPos*2+0]=_mm_max_ps(bMin.m,tt[axis].m);

			nodeStack[stackPos]=node+sign0[axis];
			node+=sign0[axis]^1;
			boxStack[stackPos*2+1]=bMax;
			stackPos++;

			tmp=_mm_unpackhi_ps(split.m,abab.m);
			tt[0].m=_mm_shuffle(0+(1<<2)+(3<<4),tmp.m);
			tt[1].m=_mm_shuffle(1+(0<<2)+(3<<4),tmp.m);
			tt[2].m=_mm_shuffle(1+(3<<2)+(0<<4),tmp.m);
			bMax=_mm_min_ps(bMax.m,tt[axis].m);
		}

		/*{
			f32x4 abab=pv[axis]*split+ov[axis];
			__m128 lo=_mm_unpacklo_ps(split.m,abab.m),hi=_mm_unpackhi_ps(split.m,abab.m);
			Vec3p pMin,pMax;

			switch(axis) {
			case 0: {
				pMin.m=_mm_shuffle(0+(1<<2)+(3<<4),lo);
				pMax.m=_mm_shuffle(0+(1<<2)+(3<<4),hi);
				break; }
			case 1: {
				pMin.m=_mm_shuffle(1+(0<<2)+(3<<4),lo);
				pMax.m=_mm_shuffle(1+(0<<2)+(3<<4),hi);
				break; }
			case 2: {
				pMin.m=_mm_shuffle(1+(3<<2)+(0<<4),lo);
				pMax.m=_mm_shuffle(1+(3<<2)+(0<<4),hi);
				break; }
			}

			nodeStack[stackPos]=node+sign0[axis];
			node+=sign0[axis]^1;
			boxStack[stackPos*2+0]=VMax(pMin,bMin);
			boxStack[stackPos*2+1]=bMax;
			bMax=Min(bMax,pMax);
			stackPos++;
		}*/
	}
}

template <class Group>
inline int KDTree::GetDepth(Group &group,const RaySelector<Group::size> &sel) const
{
	int xSign=sel.SignMas.x,ySign=sel.SignMas.y,zSign=sel.SignMas.z;
	int sign0[3]={xSign==0,ySign==0,zSign==0};
	
	Vec3p negMask; int negMaskI[3]; {
		float negMaskXYZ[4];
		negMaskXYZ[0]=xSign?-1.0f:1.0f;
		negMaskXYZ[1]=ySign?-1.0f:1.0f;
		negMaskXYZ[2]=zSign?-1.0f:1.0f;
		negMaskXYZ[3]=0.0f;
		negMaskI[0]=xSign?0x80000000:0;
		negMaskI[1]=ySign?0x80000000:0;
		negMaskI[2]=zSign?0x80000000:0;
		Convert(negMaskXYZ,negMask);
	}

	// Minimized / maximized ray origin
	Vec3p minOR,maxOR; {
		ComputeMinMaxOrigin(sel,group,minOR,maxOR);
		minOR*=negMask; maxOR*=negMask;
		Vec3p tMin=VMin(minOR,maxOR);
		maxOR=VMax(minOR,maxOR); minOR=tMin;	
	}

	f32x4 pv[3],ov[3];
	ComputePV(sel,&group.Dir(0),pv);
	ComputeOV(minOR,maxOR,pv,ov);

	Vec3p boxStack[(MaxLevel+8)*2];
	const KDNode *nodeStack[MaxLevel+8];
	int depthStack[MaxLevel+8];
	u32 stackPos=0;

	const KDNode *node=&nodes[0];
	const u32 *objIds=&objectIds[0];
	const Object *objs=&objects[0];

	Vec3p bMin,bMax; {
		Vec3p tMin=pMin*negMask,tMax=pMax*negMask;
		Vec3p tpMin=VMin(tMin,tMax),tpMax=VMax(tMin,tMax);
		bMin=VMax(minOR,tpMin);
		bMax=tpMax;
	}

	Vec3p beamDir,beamOrig; float beamM,beamA; {
		beamDir=Vec3p(0,0,0);
		Vec3q tmp=group.Dir(sel[0]);
		for(int n=1;n<sel.Num();n++) tmp+=group.Dir(sel[n]);
		Vec3p t[4]; Convert(tmp,t);
		beamDir=t[0]+t[1]+t[2]+t[3];
		beamDir*=RSqrt(beamDir|beamDir);
		beamOrig=(minOR+maxOR)*0.5f*negMask;
		Convert(Sqrt((maxOR-minOR)|(maxOR-minOR)),beamA);
		beamA*=0.5f;

		floatq minDot=Const<floatq,1>();
		Vec3q maxVec=group.Dir(sel[0]),mid=Vec3q(beamDir);

		for(int n=0;n<sel.Num();n++) {
			Vec3q &vec=group.Dir(sel[n]);
			floatq dot=vec|mid;
			typename Vec3q::TBool mask=dot<minDot;
			maxVec=Condition(mask,vec,maxVec);
			minDot=Min(dot,minDot);
		}

		floatq maxDist=Length(maxVec-mid*minDot);

		{ float t[4]; Convert(maxDist,t); beamM=Max(Max(t[0],t[1]),Max(t[2],t[3])); }
	}
	int depth=0;

	while(true) {
		u32 axis=node->Axis();
		if(axis==3) { // Leaf
			if(node->NumObjects()) {

				const u32 *oid=objIds+node->FirstObject();
				
				Vec3p nodeMin,nodeMax; {
					Vec3p nodeEps(Const<floatq,1,1000>().m);
					Vec3p tMin=bMin*negMask,tMax=bMax*negMask;
					nodeMin=VMin(tMin,tMax)-nodeEps;
					nodeMax=VMax(tMin,tMax)+nodeEps;
				}

				for(int n=node->NumObjects();n>0;n--) {
					int tid=*oid++;
					const Object &obj=objs[tid];
				//	const bool fullInNode=obj.FullInNode();
					
					bool beamCollision=obj.BeamCollide(beamOrig,beamDir,beamM,beamA);
				 	if(beamCollision)
						return depth;
				}
			}
			if(stackPos==0)
				return 255;

			stackPos--;
			bMin=boxStack[stackPos*2+0];
			bMax=boxStack[stackPos*2+1];
			node=nodeStack[stackPos];
			depth=depthStack[stackPos];
			continue;
		}

		f32x4 split; {
			union { float f; int i; };
			f=node->Pos();
			i^=negMaskI[axis];
			split=f;
		}
		node+=node->ChildDist();
		depth++;

		u32 mask=1<<axis;
		if(_mm_movemask_ps(_mm_cmpgt_ps(bMin.m,split.m))&mask) { node+=sign0[axis]; continue; }
		if(_mm_movemask_ps(_mm_cmplt_ps(bMax.m,split.m))&mask) { node+=sign0[axis]^1; continue; }

		{
			f32x4 abab=pv[axis]*split+ov[axis],tmp,tt[3];
			tmp=_mm_unpacklo_ps(split.m,abab.m);
			tt[0].m=_mm_shuffle(0+(1<<2)+(3<<4),tmp.m);
			tt[1].m=_mm_shuffle(1+(0<<2)+(3<<4),tmp.m);
			tt[2].m=_mm_shuffle(1+(3<<2)+(0<<4),tmp.m);
			boxStack[stackPos*2+0]=_mm_max_ps(bMin.m,tt[axis].m);

			depthStack[stackPos]=depth;
			nodeStack[stackPos]=node+sign0[axis];
			node+=sign0[axis]^1;
			boxStack[stackPos*2+1]=bMax;
			stackPos++;

			tmp=_mm_unpackhi_ps(split.m,abab.m);
			tt[0].m=_mm_shuffle(0+(1<<2)+(3<<4),tmp.m);
			tt[1].m=_mm_shuffle(1+(0<<2)+(3<<4),tmp.m);
			tt[2].m=_mm_shuffle(1+(3<<2)+(0<<4),tmp.m);
			bMax=_mm_min_ps(bMax.m,tt[axis].m);
		}
	}
}


template <class Output,class Group>
void KDTree::TraverseMonoGroup(Group &group,const RaySelector<Group::size> &sel,const Output &out) const {
	Vec3p orig[4],dir[4];
	u32 tmp[4];

//	for(int n=0;n<sel.Num();n++) {
//		int q=sel[n];
//		u32 *objId=out.object+q*4;
//		objId[0]=objId[1]=objId[2]=objId[3]=0;
//	}
//	return;

	if(Group::singleOrigin)
		Convert(group.Origin(sel[0]),orig);
	
	for(int i=0;i<sel.Num();i++) {
		int q=sel[i];
		Vec3p dir[4];

		if(!Group::singleOrigin)
			Convert(group.Origin(q),orig);
		Convert(group.Dir(q),dir);

		float *dist=(float*)(out.dist+q);
		u32 *objId=Output::objectIndexes?(u32*)(out.object+q):tmp;

		TraverseMono(orig[0],dir[0],NormalOutput<float,u32>(dist+0,objId+0,out.stats));
		TraverseMono(orig[1],dir[1],NormalOutput<float,u32>(dist+1,objId+1,out.stats));
		TraverseMono(orig[2],dir[2],NormalOutput<float,u32>(dist+2,objId+2,out.stats));
		TraverseMono(orig[3],dir[3],NormalOutput<float,u32>(dist+3,objId+3,out.stats));
	}
}

template <class Output,class Group,class Selector>
void KDTree::TraverseOptimized(Group &group,const Selector &sel,const floatq &maxD,const Output &out,bool primary) const {
	if(!sel.Num()) return;

	RaySelector<Group::size> selectors[9];
	group.GenSelectors(sel,selectors);

/*	int depth=primary&&Group::recLevel==2?GetDepth(group,sel):255;
	if(depth<22) {
		typedef RayGroup<1,Group::singleOrigin> GroupP;
		RaySelector<GroupP::size> tsel;
		for(int n=0;n<4;n++) tsel.Add(n);
		
		for(int k=0;k<16;k+=4) {
			GroupP gr(&group.Dir(k),&group.Origin(k));
			TraverseOptimized<NormalOutput,GroupP,RaySelector<GroupP::size> >(
				gr,tsel,maxD,NormalOutput(out.dist+k,out.object+k*4),true);
		}
		return;
	}*/

	for(int k=0;k<8;k++) {
		RaySelector<Group::size> &sel=selectors[k];
		if(sel.Num()) {
			// Ulepszyc, ta czworka moze byc popsuta (wektorki w roznych kierunkach)
			Vec3q &dir0=group.Dir(sel[0]);

			if(!primary) for(int i=1;i<sel.Num();i++) {
				floatq dot=group.Dir(sel[i])|dir0;
				if(ForAny(dot<Const<floatq,998,1000>())) {
					selectors[8].Add(sel[i]);
					sel.Disable(i--);
				}
			}

			if(sel.Num()>2) TraverseFast(group,sel,maxD,out);
			else while(sel.Num()) { selectors[8].Add(sel[0]); sel.Disable(0); }
		}
	}
	if(selectors[8].Num()) {
		const RaySelector<Group::size> &sel=selectors[8];
		TraverseMonoGroup(group,sel,out);
	}
}

#endif

