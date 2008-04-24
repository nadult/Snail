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
	struct Locals4 { float tMin,tMax; const KDNode *node; };
	Locals4 stackBegin[MaxLevel+2],*stack=stackBegin;

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
			
					if(ret>ttMin&&ret<Min(minRet,ttMax)) {
						if(Output::objectIndexes) out.object[0]=tid;
						minRet=ret;
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

	yx[0]=zx[0]=xy[0]=zy[0]=xz[0]=yz[0]=floatq(1.0f/0.0f);  // +inf
	yx[1]=zx[1]=xy[1]=zy[1]=xz[1]=yz[1]=floatq(-1.0f/0.0f); // -inf

	for(int id=0;id<sel.Num();id++) {
		const Vec3q dir=VAbs(rDir[sel[id]]);
		f32x4b mask=sel.Mask(sel[id]);

		floatq t1,t2,i;
		i=Inv(dir.x); t1=dir.y*i; t2=dir.z*i;
		yx[0]=Condition(t1<yx[0]&&mask,t1,yx[0]);
		yx[1]=Condition(t1>yx[1]&&mask,t1,yx[1]);
		zx[0]=Condition(t2<zx[0]&&mask,t2,zx[0]);
		zx[1]=Condition(t2>zx[1]&&mask,t2,zx[1]);

		i=Inv(dir.y); t1=dir.x*i; t2=dir.z*i;
		xy[0]=Condition(t1<xy[0]&&mask,t1,xy[0]);
		xy[1]=Condition(t1>xy[1]&&mask,t1,xy[1]);
		zy[0]=Condition(t2<zy[0]&&mask,t2,zy[0]);
		zy[1]=Condition(t2>zy[1]&&mask,t2,zy[1]);

		i=Inv(dir.z); t1=dir.x*i; t2=dir.y*i;
		xz[0]=Condition(t1<xz[0]&&mask,t1,xz[0]);
		xz[1]=Condition(t1>xz[1]&&mask,t1,xz[1]);
		yz[0]=Condition(t2<yz[0]&&mask,t2,yz[0]);
		yz[1]=Condition(t2>yz[1]&&mask,t2,yz[1]);
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
		float fmax=1.0f/0.0f,fmin=-1.0f/0.0f;
		Vec3q tMin=Vec3p(fmax,fmax,fmax);
		Vec3q tMax=Vec3p(fmin,fmin,fmin);

		for(int i=0;i<sel.Num();i++) {
			int q=sel[i];

			f32x4b mask=sel.Mask(q);
			Vec3q orig=group.Origin(q);
			tMin.x=Condition(orig.x<tMin.x&&mask,orig.x,tMin.x);
			tMin.y=Condition(orig.y<tMin.y&&mask,orig.y,tMin.y);
			tMin.z=Condition(orig.z<tMin.z&&mask,orig.z,tMin.z);

			tMax.x=Condition(orig.x>tMax.x&&mask,orig.x,tMax.x);
			tMax.y=Condition(orig.y>tMax.y&&mask,orig.y,tMax.y);
			tMax.z=Condition(orig.z>tMax.z&&mask,orig.z,tMax.z);
		}

		Vec3p pMin[4],pMax[4];
		Convert(tMin,pMin);
		Convert(tMax,pMax);

		min=VMin(VMin(pMin[0],pMin[1]),VMin(pMin[2],pMin[3]));
		max=VMax(VMax(pMax[0],pMax[1]),VMax(pMax[2],pMax[3]));
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

template <class Group>
INLINE void ComputeBeam(Group &group,const RaySelector<Group::size> &sel,const Vec3p &minOR,const Vec3p &maxOR,
					Vec3p &beamDir,Vec3p &beamOrig,float &beamM,float &beamA) {
	beamDir=Vec3p(0,0,0);
	Vec3q maxVec=Condition(sel.Mask(sel[0]),group.Dir(sel[0]));

	Vec3q tmp=maxVec;
	for(int n=1;n<sel.Num();n++)
		tmp+=Condition(sel.Mask(sel[n]),group.Dir(sel[n]));

	Vec3p t[4]; Convert(tmp,t);
	beamDir=t[0]+t[1]+t[2]+t[3];
	beamDir*=RSqrt(beamDir|beamDir);
	beamOrig=(minOR+maxOR)*0.5f;
	Convert(Length((maxOR-minOR)),beamA);
	beamA*=0.5f;

	floatq minDot=Const<floatq,1>();
	Vec3q mid=Vec3q(beamDir);

	for(int n=0;n<sel.Num();n++) {
		Vec3q &vec=group.Dir(sel[n]);
		floatq dot=vec|mid;

		typename Vec3q::TBool mask=dot<minDot&&sel.Mask(sel[n]);
		maxVec=Condition(mask,vec,maxVec);
		minDot=Condition(mask,dot,minDot);
	}

	floatq maxDist=Length(maxVec-mid*minDot);
	{ float t[4]; Convert(maxDist,t); beamM=Max(Max(t[0],t[1]),Max(t[2],t[3])); }
}	

template <class Output,class Group>
inline void KDTree::TraverseFast(Group &group,const RaySelector<Group::size> &tSelector,const Output &out) const
{
	assert(tSelector.Num());

	RaySelector<Group::size> sel=tSelector;

	int xSign,ySign,zSign; {
		Vec3q &dir0=group.Dir(sel[0]);
		xSign=dir0.x[0]<0;
		ySign=dir0.y[0]<0;
		zSign=dir0.z[0]<0;
	}

	int sign0[3]={xSign==0,ySign==0,zSign==0};

#define NEG(vec) Vec3p(_mm_xor_ps((vec).m,negMask))	
	union { int negMaskI[4]; __m128 negMask; }; {
		negMaskI[0]=xSign?0x80000000:0;
		negMaskI[1]=ySign?0x80000000:0;
		negMaskI[2]=zSign?0x80000000:0;
		negMaskI[3]=0;
	}

	// Minimized / maximized ray origin
	Vec3p minOR,maxOR; {
		ComputeMinMaxOrigin(sel,group,minOR,maxOR);
		minOR=NEG(minOR); maxOR=NEG(maxOR);
		Vec3p tMin=VMin(minOR,maxOR);
		maxOR=VMax(minOR,maxOR); minOR=tMin;	
	}

	floatq pv[3],ov[3];
	ComputePV(sel,&group.Dir(0),pv);
	ComputeOV(minOR,maxOR,pv,ov);

	TreeStats stats;
	stats.TracingPacket(sel.Num()*4);

	Vec3p boxStack[(MaxLevel+2)*2];
	const KDNode *nodeStack[MaxLevel+2];
	u32 stackPos=0,axis;

	const KDNode *node=&nodes[0];
	const u32 *objIds=&objectIds[0];
	const Object *objs=&objects[0];

	Vec3p bMin,bMax; {
		Vec3p tMin=NEG(pMin),tMax=NEG(pMax);
		Vec3p tpMin=VMin(tMin,tMax),tpMax=VMax(tMin,tMax);
		bMin=VMax(minOR,tpMin);
		bMax=tpMax;
	}

	Vec3p beamDir,beamOrig; float beamM,beamA;
	if(sel.Num()>4)
		ComputeBeam(group,sel,NEG(minOR),NEG(maxOR),beamDir,beamOrig,beamM,beamA);

	ObjectIdxBuffer<8> idxBuffer;

	while(true) {
		stats.LoopIteration();

		axis=node->Axis();
		if(axis==3) { // Leaf
			if(node->NumObjects()) {

				const u32 *oid=objIds+node->FirstObject();
				
				Vec3p nodeMin,nodeMax; {
					Vec3p nodeEps(Const<floatq,1,1000>().m);
					Vec3p tMin=NEG(bMin),tMax=NEG(bMax);
					nodeMin=VMin(tMin,tMax)-nodeEps;
					nodeMax=VMax(tMin,tMax)+nodeEps;
				}

				for(int n=node->NumObjects();n>0;n--) {
					int tid=*oid++;
					const Object &obj=objs[tid];
					const bool fullInNode=obj.GetFlag2();
					
					if(idxBuffer.Find(tid)) continue;

					int beamCollision=1;
					if(sel.Num()>4) {
						beamCollision=obj.BeamCollide(beamOrig,beamDir,beamM,beamA);

					 	if(!beamCollision) { idxBuffer.Insert(tid); stats.Breaking(); continue; }

						// Visualizing beam collisions
						/*	for(int i=0;i<sel.Num();i++) {
							int q=sel[i];
							Vec3q tmp(bMin.x,bMin.y,bMin.z);
							out.dist[q]=Length(group.Origin(q)-tmp);
							out.object[q]=i32x4(0);
						}
						out.stats->Update(stats);
						return; */
						stats.NotBreaking();
					}

				//	if(beamCollision==2) stats.NotBreaking();
				//	else stats.Breaking();

					bool fullInside=1;

					for(int i=0;i<sel.Num();i++) {
						int q=sel[i];
						stats.Intersection();

						floatq ret=obj.Collide(group.Origin(q),group.Dir(q));
						f32x4b mask=(ret>Const<floatq,0>()&&ret<out.dist[q]);
						u32 msk=ForWhich(mask);

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

						out.dist[q]=Condition(mask,ret,out.dist[q]);
						sel.SetBitMask(q,sel.BitMask(q)&~msk);
					}
					
					if(fullInside) idxBuffer.Insert(tid);
				}

				for(int i=0;i<sel.Num();i++)
					if(!sel.BitMask(sel[i]))
						sel.Disable(i--);
				if(sel.Num()==0) { out.stats->Update(stats); return; }
			}

POPSTACK:
			if(stackPos==0) { out.stats->Update(stats); return; }

			stackPos--;
			bMin=boxStack[stackPos*2+0];
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
	}

#undef NEG
}

// Indexes of the corner rays
// 0:top left    1:top right    2:bottom left    3:bottom right
template <class Group>
uint GetCornerIndex(uint n) {
	if(Group::size==4) { int arr[4]={0,0,0,0}; return arr[n]; }
	if(Group::size==16) { int arr[4]={0,5,10,15}; return arr[n]; }
	if(Group::size==64) { int arr[4]={0,21,42,63}; return arr[n]; }
	if(Group::size==256) { int arr[4]={0,0,0,0}; return arr[n]; }
	return 0;
}

template <class Group>
inline float KDTree::GetDensity(Group &group,const RaySelector<Group::size> &sel,TreeStats *tStats) const {
	typedef floatq base;
	typedef Vec3q Vec;

	struct Locals4 { base tMin,tMax; const KDNode *node; };

	Vec3q rOrigin,rDir; {
		Vec3p pDir[4]; Vec3q tmp;
		rOrigin=group.Origin(sel[0]);

		tmp=group.Dir(GetCornerIndex<Group>(0)); pDir[0]=Vec3p(tmp.x[0],tmp.y[0],tmp.z[0]);
		tmp=group.Dir(GetCornerIndex<Group>(1)); pDir[1]=Vec3p(tmp.x[1],tmp.y[1],tmp.z[1]);
		tmp=group.Dir(GetCornerIndex<Group>(2)); pDir[2]=Vec3p(tmp.x[2],tmp.y[2],tmp.z[2]);
		tmp=group.Dir(GetCornerIndex<Group>(3)); pDir[3]=Vec3p(tmp.x[3],tmp.y[3],tmp.z[3]);
		Convert(pDir,rDir);
	}

	Locals4 stackBegin[MaxLevel+2],*stack=stackBegin;
	Vec invDir=VInv(rDir);

	const base *rStart=&rOrigin.x;
	const base *irDir=&invDir.x;

	base tMin=0.0f,tMax=10000.0f;
	int dSign[3]={invDir.x[0]<0?0:1,invDir.y[0]<0?0:1,invDir.z[0]<0?0:1};

	const KDNode *node=&nodes[0];
	const u32 *objIds=&objectIds[0];
	const Object *objs=&objects[0];

	TreeStats stats;
	int tests=0;
	float maxValue=0.0f;

	while(true) {
		u32 axis=node->Axis();
		stats.LoopIteration();

		if(axis==3) { // leaf
			if(node->NumObjects()) {
				const u32 *id=objIds+node->FirstObject();

				for(int n=node->NumObjects();n>0;n--) {
					int tid=*id++;
					const Object &obj=objs[tid];
					stats.Intersection();
					
					/*
					floatq ret4=obj.Collide(rOrigin,rDir);
					float ret=Max(Max(ret4[0],ret4[1]),Max(ret4[2],ret4[3]));

					floatq u,v;
					obj.Barycentric(rOrigin,rDir,u,v);
					Vec2p p[4];
					p[0]=Vec2p(u[0],v[0]);
					p[1]=Vec2p(u[1],v[1]);
					p[2]=Vec2p(u[2],v[2]);
					p[3]=Vec2p(u[3],v[3]);
					
					Vec2p pmin=VMin(VMin(p[0],p[1]),VMin(p[2],p[3]));
					Vec2p pmax=VMax(VMax(p[0],p[1]),VMax(p[2],p[3]));
					
					bool collide=pmin.x<=1.0f&&pmax.x>=0.0f&&pmin.y<=1.0f&&pmax.y>=0.0f; */

				//	if(!collide) {
						float ret=Max(Max(tMax[0],tMax[1]),Max(tMax[2],tMax[3]));
				//		collide=1;
				//	}

				 //	if(collide) {

						float e1=LengthSq(obj.Edge1());
						float e2=LengthSq(obj.Edge2());
						float e3=LengthSq(obj.Edge3());
						float sizeSq=Min(e1,Min(e2,e3));
						maxValue=Max(maxValue,ret*RSqrt(sizeSq)*(0.0005f*Group::size));
						tests++;

						if(maxValue>1.0f) {
							if(tStats) tStats->Update(stats);
							return maxValue;
						}
				//	}
				}
			}

			if(stack==stackBegin) {
				if(tStats) tStats->Update(stats);
				return maxValue;
			}
			stack--;
			tMin=stack->tMin;
			tMax=stack->tMax;
			node=stack->node;
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
}

/*
template <class Group>
inline float KDTree::GetDensity(Group &group,const RaySelector<Group::size> &sel) const
{
	struct Locals4 { float tMin,tMax; const KDNode *node; };
	Locals4 stackBegin[MaxLevel+2],*stack=stackBegin;

	Vec3p beamDir,beamOrig; float beamM,beamA; {
		Vec3p minOR,maxOR;
		ComputeMinMaxOrigin(sel,group,minOR,maxOR);
		ComputeBeam(group,sel,minOR,maxOR,beamDir,beamOrig,beamM,beamA);
	}

	Vec3p rDir=Vec3p(beamDir.x+0.000000000001f,beamDir.y+0.000000000001f,beamDir.z+0.000000000001f);
	Vec3p invDir=VInv(rDir);

	const float *rStart=(const float*)&beamOrig;
	const float *irDir=(const float*)&invDir;
	float tMin=ConstEpsilon<float>(),tMax=10000.0f;

	int signMask=SignMask(floatq(invDir.m));
	int dSign[3]={signMask&1?0:1,signMask&2?0:1,signMask&4?0:1};

	const KDNode *node=&nodes[0];
	const u32 *objIds=&objectIds[0];
	const Object *objs=&objects[0];

	while(true) {
		u32 axis=node->Axis();

		if(axis==3) { // leaf
			if(node->NumObjects()) {
				const u32 *id=objIds+node->FirstObject();

				for(int n=node->NumObjects();n>0;n--) {

					int tid=*id++;
					const Object &obj=objs[tid];
				
					float ret=obj.Collide(beamOrig,beamDir);
				 	if(ret>0.0f) {
						float e1=LengthSq(obj.Edge1());
						float e2=LengthSq(obj.Edge2());
						float e3=LengthSq(obj.Edge3());
						float sizeSq=Min(e1,Min(e2,e3));

						return ret*RSqrt(sizeSq)*(0.0005f*Group::size);
					}
				}
			}

POPSTACK:
			if(stack==stackBegin) return 0;
			stack--;
			tMin=stack->tMin;
			tMax=stack->tMax;
			node=stack->node;
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
}*/

template <class Output,class Group>
void KDTree::TraverseMonoGroup(Group &group,const RaySelector<Group::size> &sel,const Output &out) const {
	Vec3p orig[4];

	if(Group::singleOrigin)
		Convert(group.Origin(sel[0]),orig);
	
	for(int i=0;i<sel.Num();i++) {
		int q=sel[i];
		Vec3p dir[4];
		int msk=sel.BitMask(q);

		if(!Group::singleOrigin)
			Convert(group.Origin(q),orig);
		Convert(group.Dir(q),dir);

		float *dist=(float*)(out.dist+q); u32 tmp[4];
		u32 *objId=Output::objectIndexes?(u32*)(out.object+q):tmp;

		if(msk&1) TraverseMono(orig[0],dir[0],NormalOutput<float,u32>(dist+0,objId+0,out.stats));
		if(msk&2) TraverseMono(orig[1],dir[1],NormalOutput<float,u32>(dist+1,objId+1,out.stats));
		if(msk&4) TraverseMono(orig[2],dir[2],NormalOutput<float,u32>(dist+2,objId+2,out.stats));
		if(msk&8) TraverseMono(orig[3],dir[3],NormalOutput<float,u32>(dist+3,objId+3,out.stats));

	}
}

template <class Output,class Group,class Selector>
void KDTree::TraverseOptimized(Group &group,const Selector &sel,const Output &out,bool primary) const {
	if(!sel.Num()) return;

	RaySelector<Group::size> selectors[9];
	group.GenSelectors(sel,selectors);

	if(Group::recLevel==3&&primary) {
		if(GetDensity(group,sel)>1.0f) {
			typedef RayGroup<2,Group::singleOrigin> GroupP;
			RaySelector<GroupP::size> tsel;
			for(int n=0;n<16;n++) tsel.Add(n);
		
			for(int k=0;k<64;k+=16) {
				GroupP gr(&group.Dir(k),&group.Origin(k));
				TraverseOptimized(gr,tsel,NormalOutput<floatq,i32x4>(out.dist+k,out.object+k,out.stats));
			}
			return;
		}
	}
	if(Group::recLevel==2&&primary) {
		if(GetDensity(group,sel)>1.0f) {
			typedef RayGroup<1,Group::singleOrigin> GroupP;
			RaySelector<GroupP::size> tsel;
			for(int n=0;n<4;n++) tsel.Add(n);
		
			for(int k=0;k<16;k+=4) {
				GroupP gr(&group.Dir(k),&group.Origin(k));
				TraverseOptimized(gr,tsel,NormalOutput<floatq,i32x4>(out.dist+k,out.object+k,out.stats));
			}
			return;
		}
	}

	for(int k=0;k<8;k++) {
		RaySelector<Group::size> &sel=selectors[k];
		if(sel.Num()) {
			// Ulepszyc, ta czworka moze byc popsuta (wektorki w roznych kierunkach)
			Vec3q dir0=group.Dir(sel[0]);

			if(!primary) for(int i=1;i<sel.Num();i++) {
				int q=sel[i];

				int bitMask=sel.BitMask(q);
				if(CountMaskBits(bitMask)==1) { selectors[8].Add(q,bitMask); sel.Disable(i--); continue; }

				floatq dot=group.Dir(q)|dir0;
				if(ForAny(dot<Const<floatq,998,1000>())) { selectors[8].Add(q); sel.Disable(i--); continue; 	}
			}

			if(sel.Num()>2) TraverseFast(group,sel,out);
			else while(sel.Num()) { selectors[8].Add(sel[0]); sel.Disable(0); }
		}
	}
	if(selectors[8].Num()) {
		const RaySelector<Group::size> &sel=selectors[8];
		TraverseMonoGroup(group,sel,out);
	}
}

#endif

