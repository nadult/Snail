
template <class Output,class Vec,class base>
void KDTree::FullTraverse(const Vec &rOrigin,const Vec &rDir,const base &maxD,const Output &out) const {	
	if(Output::objectIdsFlag) {
		for(int m=0;m<ScalarInfo<base>::Multiplicity;m++)
			out.object[m]=0;
	}
	out.dist=maxD;

	for(int n=0;n<Object::nObjs;n++) {
		base dst=Object::spheres[n].Collide(rOrigin,rDir);
		typename Vec::TBool col=dst>Const<base,0>::Value()&&dst<out;

		if(Output::objectIdsFlag) {
			u32 msk=ForWhich(col);
			for(int m=0;m<ScalarInfo<base>::Multiplicity;m++)
				if(msk&(1<<m)) out.object[m]=n;
		}
		out.dist=Condition(col,dst,out);
	}
}

INLINE void KDTree::TraverseMono(int packetId,const Vec3p &rOrigin,const Vec3p &tDir,const float &maxD,float &dist,u32 *outObj) const
{
	struct Locals4 {
		float tMin,tMax;
		const KDNode *node;
	};
	Locals4 stackBegin[MaxLevel+16],*stack=stackBegin;

	Vec3p rDir=Vec3p(	tDir.X()+0.000000000001f,
						tDir.Y()+0.000000000001f,
						tDir.Z()+0.000000000001f);
	Vec3p invDir=Inv(rDir);

	const float *rStart=(const float*)&rOrigin;
	const float *irDir=(const float*)&invDir;
	float tMin=ConstEpsilon<float>::Value(),tMax=maxD;

	*outObj=0;

	int dSign[3];
	GetVecSigns(invDir,dSign);
	float minRet=maxD;

	const KDNode *node=&nodes[0];
	const u32 *objIds=&objectIds[0];
	const Object *objs=&objects[0];

	KDStats localStats;

	while(true) {
		localStats.iters++;
		u32 axis=node->Axis();

		if(axis==3) { // leaf
			if(node->NumObjects()) {
				const u32 *id=objIds+node->FirstObject();

				float eps=0.0001f,ttMin=tMin-eps,ttMax=tMax+eps;
				
				for(int n=node->NumObjects();n>0;n--) {
					int tid=*id++;
					const Object &obj=objs[tid];
					float ret; Convert(obj.Collide(rOrigin,rDir),ret);
					
					bool mask=ret>ttMin&&ret<Min(minRet,ttMax);

					localStats.colTests++;
					if(mask) *outObj=tid;

					minRet=mask?ret:minRet;
				}
				if(minRet<maxD) {
					stats.Update(localStats);
					dist=minRet;
					return;
				}
			}

			if(stack==stackBegin) {
				stats.Update(localStats);
				dist=minRet;
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
	SSEReal yx[2],zx[2],xy[2],zy[2],xz[2],yz[2];

	{
		const Vec3q dir=Abs(rDir[sel[0]]);

		SSEReal ix=Inv(dir.X()),iy=Inv(dir.Y()),iz=Inv(dir.Z());
		yx[0]=yx[1]=dir.Y()*ix;
		zx[0]=zx[1]=dir.Z()*ix;
		xy[0]=xy[1]=dir.X()*iy;
		zy[0]=zy[1]=dir.Z()*iy;
		xz[0]=xz[1]=dir.X()*iz;
		yz[0]=yz[1]=dir.Y()*iz;
	}

	for(int id=1;id<sel.Num();id++) {
		const Vec3q dir=Abs(rDir[sel[id]]);

		SSEReal t1,t2,i;
		i=Inv(dir.X()); t1=dir.Y()*i; t2=dir.Z()*i;
		yx[0]=Min(t1,yx[0]); yx[1]=Max(t1,yx[1]);
		zx[0]=Min(t2,zx[0]); zx[1]=Max(t2,zx[1]);

		i=Inv(dir.Y()); t1=dir.X()*i; t2=dir.Z()*i;
		xy[0]=Min(t1,xy[0]); xy[1]=Max(t1,xy[1]);
		zy[0]=Min(t2,zy[0]); zy[1]=Max(t2,zy[1]);

		i=Inv(dir.Z()); t1=dir.X()*i; t2=dir.Y()*i;
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
		Vec3p tMin=Min(Min(pOR[0],pOR[1]),Min(pOR[2],pOR[3]));
		Vec3p tMax=Max(Max(pOR[0],pOR[1]),Max(pOR[2],pOR[3]));
		for(int i=1;i<sel.Num();i++) {
			int q=sel[i];
			Convert(group.Origin(q),pOR);
			tMin=Min( Min(Min(pOR[0],pOR[1]),Min(pOR[2],pOR[3])), tMin);
			tMax=Max( Max(Max(pOR[0],pOR[1]),Max(pOR[2],pOR[3])), tMax);
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
inline void KDTree::TraverseFast(int packetId,Group &group,const RaySelector<Group::size> &tSelector,
								 const floatq &maxD,const Output &out) const
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
		if(Output::objectIdsFlag) {
			for(int n=0;n<ScalarInfo<floatq>::Multiplicity;n++)
				out.object[q*4+n]=0;
		}
	}
//	if(!(negMaskI[0]!=0&&negMaskI[1]!=0&&negMaskI[2]==0))
//		return;

	// Minimized / maximized ray origin
	Vec3p minOR,maxOR; {
		ComputeMinMaxOrigin(sel,group,minOR,maxOR);
		minOR*=negMask; maxOR*=negMask;
		Vec3p tMin=Min(minOR,maxOR);
		maxOR=Max(minOR,maxOR); minOR=tMin;	
	}

	SSEReal pv[3],ov[3];
	ComputePV(sel,&group.Dir(0),pv);
	ComputeOV(minOR,maxOR,pv,ov);

	KDStats localStats;

	Vec3p boxStack[(MaxLevel+8)*2];
	const KDNode *nodeStack[MaxLevel+8];
	u32 stackPos=0;

	char active[Group::size];
	for(int i=0;i<sel.Num();i++)
		active[i]=15;

	const KDNode *node=&nodes[0];
	const u32 *objIds=&objectIds[0];
	const Object *objs=&objects[0];

	Vec3p bMin,bMax; {
		Vec3p tMin=pMin*negMask,tMax=pMax*negMask;
		Vec3p tpMin=Min(tMin,tMax),tpMax=Max(tMin,tMax);
		bMin=Max(minOR,tpMin);
		bMax=tpMax;
	}

	Vec3p midDir; {
		midDir=Vec3p(0,0,0);
		Vec3q tmp=group.Dir(sel[0]);
		for(int n=1;n<sel.Num();n++)
			tmp+=group.Dir(sel[n]);
		Vec3p t[4]; Convert(tmp,t);
		midDir=t[0]+t[1]+t[2]+t[3];
		midDir*=RSqrt(midDir|midDir);
	}

	while(true) {
		localStats.iters++;

		u32 axis=node->Axis();
		if(axis==3) { // Leaf
			if(node->NumObjects()) {

				const u32 *oid=objIds+node->FirstObject();
				
				SSEPVec3 nodeMin,nodeMax; {
					SSEPVec3 nodeEps(Const<floatq,1,10000>::Value().m);
					Vec3p tMin=bMin*negMask,tMax=bMax*negMask;
					nodeMin=Min(tMin,tMax)-nodeEps;
					nodeMax=Max(tMin,tMax)+nodeEps;
				}

				for(int n=node->NumObjects();n>0;n--) {
					int tid=*oid++;
					const Object &obj=objs[tid];
					const bool fullInNode=obj.fullInNode;
					
					if(obj.lastVisit==packetId) { /*localStats.skips++;*/ continue; }

					bool beamCollision=obj.BeamCollide(pv,ov,negMask,
							(minOR+maxOR)*floatp(0.5f),midDir);
						//	minOR,maxOR);
					/*{
						floatq ret1=obj.Collide(group.Origin(0),group.Dir(0));
						floatq ret2=obj.Collide(group.Origin(sel.Num()-1),group.Dir(sel.Num()-1));
						SSEMask mask=	(ret1>Const<floatq,0>::Value()&&ret1<out.dist[0])||
										(ret2>Const<floatq,0>::Value()&&ret2<out.dist[0]);
						beamCollision=ForAny(mask);
					}*/

				 	if(!beamCollision) {
						obj.lastVisit=packetId;
						localStats.skips++;
						continue;
					}
					bool fullInside=1;

					int anyPassed=0;

					for(int i=0;i<sel.Num();i++) {
						int q=sel[i];
						localStats.colTests++;

						floatq ret=obj.Collide(group.Origin(q),group.Dir(q));
						SSEMask mask=(ret>Const<floatq,0>::Value()&&ret<out.dist[q]);
						u32 msk=ForWhich(mask);

						anyPassed+=msk;

						if(!fullInNode) {
							SSEVec3 col=group.Origin(q)+group.Dir(q)*ret;
							SSEMask insideMask=	col.X()>=nodeMin.X()&&col.X()<=nodeMax.X()&&
												col.Y()>=nodeMin.Y()&&col.Y()<=nodeMax.Y()&&
												col.Z()>=nodeMin.Z()&&col.Z()<=nodeMax.Z();
							u32 insideMsk=ForWhich(insideMask);
							mask=mask&&insideMask; 
							// Jakies promienie uderzyly w obiekt poza KD-nodem
							if(msk^insideMsk) fullInside=0;
							msk&=insideMsk;
						}

			//			if(msk&&!beamCollision) {
			//				throw Exception("Error!!!");
			//			}

						if(Output::objectIdsFlag) {
							if(msk) for(int m=0;m<ScalarInfo<floatq>::Multiplicity;m++)
								if(msk&(1<<m)) out.object[q*4+m]=tid;
						}

						active[i]^=msk;
						out.dist[q]=Condition(mask,ret,out.dist[q]);
					}

					if(!anyPassed) localStats.intersectOk++;
					else localStats.intersectFail++;

					if(fullInside) {
						obj.lastVisit=packetId;
						localStats.notBreaking++;
					}
					else localStats.breaking++;
				}

				for(int i=0;i<sel.Num();i++) if(!active[i]) {
					active[i]=active[sel.Num()-1];
					sel.Disable(i--);
				}
				if(sel.Num()==0) { stats.Update(localStats); return; }
			}
			if(stackPos==0) { stats.Update(localStats); return; }

			stackPos--;
			bMin=boxStack[stackPos*2+0];
			bMax=boxStack[stackPos*2+1];
			node=nodeStack[stackPos];
			continue;
		}

		SSEReal split; {
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
			SSEReal abab=pv[axis]*split+ov[axis],tmp,tt[3];
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
			Vec3p pMin,pMax;

			switch(axis) {
			case 0: {
				SSEReal yzyz=pv[0]*split+ov[0];
				pMin.m=_mm_shuffle(0+(1<<2)+(3<<4),_mm_unpacklo_ps(split.m,yzyz.m));
				pMax.m=_mm_shuffle(0+(1<<2)+(3<<4),_mm_unpackhi_ps(split.m,yzyz.m));
				break; }
			case 1: {
				SSEReal xzxz=pv[1]*split+ov[1];
				pMin.m=_mm_shuffle(1+(0<<2)+(3<<4),_mm_unpacklo_ps(split.m,xzxz.m));
				pMax.m=_mm_shuffle(1+(0<<2)+(3<<4),_mm_unpackhi_ps(split.m,xzxz.m));
				break; }
			case 2: {
				SSEReal xyxy=pv[2]*split+ov[2];
				pMin.m=_mm_shuffle(1+(3<<2)+(0<<4),_mm_unpacklo_ps(split.m,xyxy.m));
				pMax.m=_mm_shuffle(1+(3<<2)+(0<<4),_mm_unpackhi_ps(split.m,xyxy.m));
				break; }
			}

			nodeStack[stackPos]=node+sign0[axis];
			node+=sign0[axis]^1;
			boxStack[stackPos*2+0]=Max(pMin,bMin);
			boxStack[stackPos*2+1]=bMax;
			bMax=Min(bMax,pMax);
			stackPos++;
		}*/
	}
}

template <class Output,class Group>
void KDTree::TraverseOptimized(int packetId,Group &group,const RaySelector<Group::size> &sel,
							   const floatq &maxD,const Output &out,bool primary) const {
	if(!sel.Num()) return;

	RaySelector<Group::size> selectors[9];

	group.GenSelectors(sel,selectors);

	for(int k=0;k<8;k++) if(selectors[k].Num()) {
	/*	if(primary&&selectors[k].Num()==sel.Num()) {
			int num=sel.Num();
			float fmaxD[4]; Convert(maxD,fmaxD);
			float dist; u32 obj,tObj;

			Vec3p orig[4]; Vec3p dir[4];
			Convert(group.Origin(0),orig); Convert(group.Dir(0),dir);
			TraverseMono(0,orig[0],dir[0],fmaxD[0],dist,&obj);
			if(obj==0) goto NORMAL_TRAV;
			tObj=obj; Convert(group.Origin(num-1),orig); Convert(group.Dir(num-1),dir);
			TraverseMono(0,orig[3],dir[3],fmaxD[0],dist,&obj);
			if(obj!=tObj) goto NORMAL_TRAV;

			const Object &object=objects[obj];
			for(int n=0;n<num;n++) {
				floatq dist=object.Collide(group.Origin(n),group.Dir(n));
				out.dist[n]=dist;
				for(int i=0;i<4;i++) out.object[n*4+i]=obj;
			}
			continue;
		}

NORMAL_TRAV:*/
		TraverseFast(PacketIdGenerator::Gen(packetId),group,selectors[k],maxD,out);
		stats.runs++; stats.coherent+=sel.Num();
	}
	if(selectors[8].Num()) {
		RaySelector<Group::size> sel=selectors[8];

		/* for(int i=0;i<sel.Num();i++) {
			int q=sel[i];
			Traverse(PacketIdGenerator::Gen(packetId),group.Origin(q),group.Dir(q),maxD,Output(out,q));
		}*/
		
		for(int i=0;i<sel.Num();i++) {
			int q=sel[i];
			Vec3p orig[4]; Vec3p dir[4]; float fmaxD[4];
			Convert(group.Origin(q),orig);
			Convert(group.Dir(q),dir);
			Convert(maxD,fmaxD);

			float *dist=(float*)(out.dist+q);
			TraverseMono(0,orig[0],dir[0],fmaxD[0],dist[0],out.object+q*4+0);
			TraverseMono(0,orig[1],dir[1],fmaxD[1],dist[1],out.object+q*4+1);
			TraverseMono(0,orig[2],dir[2],fmaxD[2],dist[2],out.object+q*4+2);
			TraverseMono(0,orig[3],dir[3],fmaxD[3],dist[3],out.object+q*4+3);
		}
		stats.runs+=sel.Num();
		stats.nonCoherent+=sel.Num();
	}
	stats.tracedRays+=sel.Num()*4;
}



/*
 * Disabled, because it needs bounding boxes
 * for every kd node
 */
/*
template <class Output,class Vec,class base>
void KDTree::Traverse(int packetId,const Vec &rOrigin,const Vec &rDir,const base &maxD,const Output &out) const
{
	struct Locals4 {
		base tMin,tMax;
		const KDNode *node;
	};

	Locals4 stackBegin[MaxLevel+16],*stack=stackBegin;
	Vec invDir=Inv(rDir);

	const base *rStart=&rOrigin.X();
	const base *irDir=&invDir.X();

	base tMin=ConstEpsilon<base>::Value(),tMax=maxD;

	if(Output::objectIdsFlag) {
		for(int n=0;n<ScalarInfo<base>::Multiplicity;n++)
			out.object[n]=0;
	}

	int dSign[3];
	GetVecSigns(invDir,dSign);

	base minRet=maxD;

	const KDNode *node=&nodes[0];
	const u32 *objIds=&objectIds[0];
	const Object *objs=&objects[0];

	KDStats localStats;

	while(true) {
		localStats.iters++;
		u32 axis=node->Axis();

		if(axis==3) { // leaf
			if(node->NumObjects()) {
				const u32 *id=objIds+node->FirstObject();
				
				SSEPVec3 nodeMin,nodeMax; {
					int nodeIdx=node-&nodes[0];
					SSEPVec3 nodeEps(Const<floatq,1,1000>::Value().m);
					nodeMin=nodeMinMax[nodeIdx*2+0]-nodeEps;
					nodeMax=nodeMinMax[nodeIdx*2+1]+nodeEps;
				}

				for(int n=node->NumObjects();n>0;n--) {
					int tid=*id++;
					const Object &obj=objs[tid];
					base ret=obj.Collide(rOrigin,rDir);
					
					if(obj.lastVisit==packetId) { localStats.skips++; continue; }
					bool fullInside=1;
					
					typename Vec::TBool mask=ret>Const<base,0>::Value()&&ret<minRet;
					u32 msk=ForWhich(mask);	
	
					if(!obj.fullInNode) {
						Vec col=rOrigin+rDir*ret;
						typename Vec::TBool insideMask=	col.X()>=nodeMin.X()&&col.X()<=nodeMax.X()&&
														col.Y()>=nodeMin.Y()&&col.Y()<=nodeMax.Y()&&	
														col.Z()>=nodeMin.Z()&&col.Z()<=nodeMax.Z();
						
						u32 insideMsk=ForWhich(insideMask);
						mask=mask&&insideMask; 
						// Jakies promienie uderzyly w obiekt poza KD-nodem
						if(msk^insideMsk) fullInside=0;
						msk&=insideMsk;
					}

					localStats.colTests++;

					if(Output::objectIdsFlag) {
						if(msk) for(int m=0;m<ScalarInfo<base>::Multiplicity;m++)
							if(msk&(1<<m)) out.object[m]=tid;
					}

					if(fullInside) {
						obj.lastVisit=packetId;
						stats.notBreaking++;
					}
					else stats.breaking++;

					minRet=Condition(mask,ret,minRet);
				}

				if(ForAll(minRet<maxD)) {
					stats.Update(localStats);
					out.dist[0]=minRet;
					return;
				}

			}

			if(stack==stackBegin) {
				stats.Update(localStats);
				out.dist[0]=minRet;
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

		base tSplit=base(node->Pos()-rStart[axis])*irDir[axis];
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
}*/
