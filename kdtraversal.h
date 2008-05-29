#ifndef RTRACER_KDTRAVERSAL_H
#define RTRACER_KDTRAVERSAL_H

// Indexes of the corner rays
// 0:top left    1:top right    2:bottom left    3:bottom right
template <int packetSize>
uint GetCornerIndex(uint n) {
	if(packetSize==4) { int arr[4]={0,1,2,3}; return arr[n]; }
	if(packetSize==16) { int arr[4]={0,5,10,15}; return arr[n]; }
	if(packetSize==64) { int arr[4]={0,21,42,63}; return arr[n]; }
	if(packetSize==256) { int arr[4]={0,0,0,0}; return arr[n]; }
	return 0;
}

/*!
	(Absolute)
	px = [ dy[0].y/dy[0].x	dz[0].z/dz[0].x		dy[1].y/dy[1].x		dz[1].z/dz[1].x	]
	py = [ dx[0].x/dx[0].y	dz[0].z/dz[0].y		dx[1].x/dx[1].y		dz[1].z/dz[1].y	]
	pz = [ dx[0].x/dx[0].z	dy[0].y/dy[0].z		dx[1].x/dx[1].z		dy[1].y/dy[1].z	]
*/
template <class Rays,class Selector>
INLINE void ComputePV(Rays &rays,const Selector &sel,floatq *pv)
{
	//	ab[k] = k==0? min(a/b)  :   max(a/b)
	floatq yx[2],zx[2],xy[2],zy[2],xz[2],yz[2];

	yx[0]=zx[0]=xy[0]=zy[0]=xz[0]=yz[0]=floatq(+1.0f/0.0f); // +inf
	yx[1]=zx[1]=xy[1]=zy[1]=xz[1]=yz[1]=floatq(-1.0f/0.0f); // -inf

#define INSERT(id) { int tid=id; \
		const Vec3q dir=VAbs(rays.Dir(tid)); \
		Vec3q idir=VAbs(rays.IDir(tid)); \
	\
		floatq t1,t2; \
		t1=dir.y*idir.x; t2=dir.z*idir.x; \
		yx[0]=Min(yx[0],t1); \
		yx[1]=Max(yx[1],t1); \
		zx[0]=Min(zx[0],t2); \
		zx[1]=Max(zx[1],t2); \
\
		t1=dir.x*idir.y; t2=dir.z*idir.y; \
		xy[0]=Min(xy[0],t1); \
		xy[1]=Max(xy[1],t1); \
		zy[0]=Min(zy[0],t2); \
		zy[1]=Max(zy[1],t2); \
\
		t1=dir.x*idir.z; t2=dir.y*idir.z; \
		xz[0]=Min(xz[0],t1); \
		xz[1]=Max(xz[1],t1); \
		yz[0]=Min(yz[0],t2); \
		yz[1]=Max(yz[1],t2); \
	}

	bool primary=1;
	if(primary&&Selector::size==sel.Num()) {
		int block=sel[0]&~(Selector::size-1);
		INSERT(GetCornerIndex<Selector::size>(0)+block);
		INSERT(GetCornerIndex<Selector::size>(1)+block);
		INSERT(GetCornerIndex<Selector::size>(2)+block);
		INSERT(GetCornerIndex<Selector::size>(3)+block);
	}
	else for(int id=0;id<sel.Num();id++) {
		int tid=sel[id];
		const Vec3q dir=VAbs(rays.Dir(tid));
		Vec3q idir=VAbs(rays.IDir(tid));
		f32x4b mask=sel.Mask(id);
	
		floatq t1,t2;
		t1=dir.y*idir.x; t2=dir.z*idir.x;
		yx[0]=Condition(t1<yx[0]&&mask,t1,yx[0]);
		yx[1]=Condition(t1>yx[1]&&mask,t1,yx[1]);
		zx[0]=Condition(t2<zx[0]&&mask,t2,zx[0]);
		zx[1]=Condition(t2>zx[1]&&mask,t2,zx[1]);

		t1=dir.x*idir.y; t2=dir.z*idir.y;
		xy[0]=Condition(t1<xy[0]&&mask,t1,xy[0]);
		xy[1]=Condition(t1>xy[1]&&mask,t1,xy[1]);
		zy[0]=Condition(t2<zy[0]&&mask,t2,zy[0]);
		zy[1]=Condition(t2>zy[1]&&mask,t2,zy[1]);

		t1=dir.x*idir.z; t2=dir.y*idir.z;
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

template <class Rays,class Selector>
INLINE void ComputeMinMaxOrigin(const Rays &rays,const Selector &sel,Vec3p &min,Vec3p &max) {
	if(Rays::sharedOrigin) {
		Vec3p pOR[4]; Convert(rays.Origin(sel[0]),pOR);
		min=max=pOR[0];
	}
	else {
		float fmax=1.0f/0.0f,fmin=-1.0f/0.0f;
		Vec3q tMin=Vec3p(fmax,fmax,fmax);
		Vec3q tMax=Vec3p(fmin,fmin,fmin);

		for(int i=0;i<sel.Num();i++) {
			int q=sel[i];

			f32x4b mask=sel.Mask(i);
			Vec3q orig=rays.Origin(q);
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

template <class Rays,class Selector>
void ComputeBeam(Rays &rays,const Selector &sel,const Vec3p &minOR,const Vec3p &maxOR,
					Vec3p &beamDir,Vec3p &beamOrig,float &beamM,float &beamA) NOINLINE;

template <class Rays,class Selector>
void ComputeBeam(Rays &rays,const Selector &sel,const Vec3p &minOR,const Vec3p &maxOR,
					Vec3p &beamDir,Vec3p &beamOrig,float &beamM,float &beamA) {
	beamDir=Vec3p(0,0,0);
	Vec3q maxVec=Condition(sel.Mask(0),rays.Dir(sel[0]));

	Vec3q tmp=maxVec;
	for(int n=1;n<sel.Num();n++)
		tmp+=Condition(sel.Mask(n),rays.Dir(sel[n]));

	Vec3p t[4]; Convert(tmp,t);
	beamDir=t[0]+t[1]+t[2]+t[3];
	beamDir*=RSqrt(beamDir|beamDir);
	beamOrig=(minOR+maxOR)*0.5f;
	Convert(Length((maxOR-minOR)),beamA);
	beamA*=0.5f;

	floatq minDot=Const<floatq,1>();
	Vec3q mid=Vec3q(beamDir);

	for(int n=0;n<sel.Num();n++) {
		Vec3q &vec=rays.Dir(sel[n]);
		floatq dot=vec|mid;

		typename Vec3q::TBool mask=dot<minDot&&sel.Mask(n);
		maxVec=Condition(mask,vec,maxVec);
		minDot=Condition(mask,dot,minDot);
	}

	floatq maxDist=Length(maxVec-mid*minDot);
	{ float t[4]; Convert(maxDist,t); beamM=Max(Max(t[0],t[1]),Max(t[2],t[3])); }
}


template <class Rays,class Selector>
class TraverseContext {
public:
	template <class OtherContext>
	TraverseContext(Rays &rays,const Selector &selector,const KDTree &tree,OtherContext *other) {
		sel=selector;

		int xSign,ySign,zSign; {
			Vec3q &dir0=rays.Dir(sel[0]);
			xSign=dir0.x[0]<0;
			ySign=dir0.y[0]<0;
			zSign=dir0.z[0]<0;
		}

		sign0[0]=xSign==0;
		sign0[1]=ySign==0;
		sign0[2]=zSign==0;
		sign1[0]=sign0[0]^1;
		sign1[1]=sign0[1]^1;
		sign1[2]=sign0[2]^1;

		negMaskI[0]=xSign?0x80000000:0;
		negMaskI[1]=ySign?0x80000000:0;
		negMaskI[2]=zSign?0x80000000:0;
		negMaskI[3]=0;

		{
			ComputeMinMaxOrigin(rays,sel,minOR,maxOR);
			minOR=Neg(minOR); maxOR=Neg(maxOR);
			Vec3p tMin=VMin(minOR,maxOR);
			maxOR=VMax(minOR,maxOR); minOR=tMin;	
		}

		ComputePV(rays,sel,pv);
		ComputeOV(minOR,maxOR,pv,ov);


		if(Selector::size>4&&sel.Num()>4)
			ComputeBeam(rays,sel,Neg(minOR),Neg(maxOR),beamDir,beamOrig,beamM,beamA);

		if(other) {
			node=other->node;
			stackPos=other->stackPos;
			for(int n=0;n<stackPos;n++) {
				boxStack[n*2+0]=other->boxStack[n*2+0];
				boxStack[n*2+1]=other->boxStack[n*2+1];
				nodeStack[n]=other->nodeStack[n];
			}
			bMin=other->bMin;
			bMax=other->bMax;
			idxBuffer=other->idxBuffer;
		}
		else {
			// Compute bMin, bMax
			Vec3p tMin=Neg(tree.pMin),tMax=Neg(tree.pMax);
			Vec3p tpMin=VMin(tMin,tMax),tpMax=VMax(tMin,tMax);
			bMin=VMax(minOR,tpMin);
			bMax=tpMax;

			node=&tree.nodes[0];
			stackPos=0;
		}
	}
	Vec3p Neg(const Vec3p &v) const { return Vec3p(_mm_xor_ps(v.m,negMask)); }

	Selector sel;

	int sign0[3],sign1[3];
	union { int negMaskI[4]; __m128 negMask; };

	Vec3p minOR,maxOR;
	floatq pv[3],ov[3];
	Vec3p beamDir,beamOrig; float beamM,beamA;
	
	Vec3p boxStack[(KDTree::MaxLevel+2)*2];
	const KDNode *nodeStack[KDTree::MaxLevel+2];
	u32 stackPos;

	const KDNode *node;
	Vec3p bMin,bMax;

	ObjectIdxBuffer<4> idxBuffer;
};


template <class Output,class Rays,class Selector,class OtherContext>
inline void KDTree::TraverseFast(Rays &rays,const Selector &tSelector,const Output &out,OtherContext *tContext) const
{
	if(tSelector.Num()<3) {
		TraverseMono(rays,tSelector,out);
		return;
	}
	bool noSplit=0;
	if(!splittingFlag) noSplit=1;

	TraverseContext<Rays,Selector> c(rays,tSelector,*this,tContext);

	const u32 *objIds=&objectIds[0];
	const KDTree::Object *objs=&objects[0];

	TreeStats stats;
	u32 axis;
	int count=0;

	while(true) {
		stats.LoopIteration();

		axis=c.node->Axis();
		if(axis==3) { // Leaf

			if(c.node->NumObjects()) {
				const u32 *oid=objIds+c.node->FirstObject();
				
				Vec3p nodeMin,nodeMax; {
					Vec3p nodeEps(Const<floatq,1,1000>().m);
					Vec3p tMin=c.Neg(c.bMin),tMax=c.Neg(c.bMax);
					nodeMin=VMin(tMin,tMax)-nodeEps;
					nodeMax=VMax(tMin,tMax)+nodeEps;
				}

				for(int n=c.node->NumObjects();n>0;n--) {
					int tid=*oid++;
					const Object &obj=objs[tid];
					const bool fullInNode=obj.GetFlag2();
					
					if(c.idxBuffer.Find(tid)) continue;

					int beamCollision=1;
					if(Selector::size>4&&c.sel.Num()>4) {
						Vec3p colPos;

						beamCollision=obj.BeamCollide(c.beamOrig,c.beamDir,c.beamM,c.beamA,&colPos);
					 	if(!beamCollision) { c.idxBuffer.Insert(tid); continue; }

						// Visualizing beam collisions
					/*	for(int i=0;i<c.sel.Num();i++) {
							int q=c.sel[i];
							Vec3q tmp=colPos;
							out.dist[q]=Length(rays.Origin(q)-tmp);
							out.object[q]=i32x4(0);
						}
						out.stats->Update(stats);
						return; */
						if(beamCollision==2) {
							stats.NotBreaking();
						}
						else {
							count++;
							stats.Breaking();
					}
					}

					bool fullInside=1;

					for(int i=0;i<c.sel.Num();i++) {
						int q=c.sel[i];
						stats.Intersection();

						floatq ret=obj.Collide(rays.Origin(q),rays.Dir(q));
						f32x4b mask=(ret>Const<floatq,0>()&&ret<out.dist[q]);
						u32 msk=ForWhich(mask);

						if(!fullInNode) {
							Vec3q col=rays.Origin(q)+rays.Dir(q)*ret;
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
						c.sel.SetBitMask(i,c.sel.BitMask(i)&~msk);
					}
	
					if(fullInside) c.idxBuffer.Insert(tid);
				}

				for(int i=0;i<c.sel.Num();i++)
					if(!c.sel.BitMask(i))
						c.sel.Disable(i--);
				if(c.sel.Num()==0) { out.stats->Update(stats); return; }
	
				if(Selector::size>4&&count>=4&&noSplit==0) {
					stats.Skip();
					out.stats->Update(stats);
					if(c.stackPos==0) { out.stats->Update(stats); return; }
					c.stackPos--;
					c.bMin=c.boxStack[c.stackPos*2+0];
					c.bMax=c.boxStack[c.stackPos*2+1];
					c.node=c.nodeStack[c.stackPos];
					TraverseSplitted(rays,c.sel,out,&c);
					return;
				}
			}

POPSTACK:
			if(c.stackPos==0) { out.stats->Update(stats); return; }

			c.stackPos--;
			c.bMin=c.boxStack[c.stackPos*2+0];
			c.bMax=c.boxStack[c.stackPos*2+1];
			c.node=c.nodeStack[c.stackPos];

			continue;
		}

		f32x4 split; {
			union { float f; int i; };
			f=c.node->Pos();
			i^=c.negMaskI[axis];
			split=f;
		}
		c.node+=c.node->ChildDist();

		u32 mask=1<<axis;
		if(_mm_movemask_ps(_mm_cmpgt_ps(c.bMin.m,split.m))&mask) { c.node+=c.sign0[axis]; continue; }
		if(_mm_movemask_ps(_mm_cmplt_ps(c.bMax.m,split.m))&mask) { c.node+=c.sign1[axis]; continue; }

		{
			f32x4 abab=c.pv[axis]*split+c.ov[axis],tmp,tt[3];
			tmp=_mm_unpacklo_ps(split.m,abab.m);
			tt[0].m=_mm_shuffle(0+(1<<2)+(3<<4),tmp.m);
			tt[1].m=_mm_shuffle(1+(0<<2)+(3<<4),tmp.m);
			tt[2].m=_mm_shuffle(1+(3<<2)+(0<<4),tmp.m);
			c.boxStack[c.stackPos*2+0]=_mm_max_ps(c.bMin.m,tt[axis].m);

			c.nodeStack[c.stackPos]=c.node+c.sign0[axis];
			c.node+=c.sign1[axis];
			c.boxStack[c.stackPos*2+1]=c.bMax;
			c.stackPos++;

			tmp=_mm_unpackhi_ps(split.m,abab.m);
			tt[0].m=_mm_shuffle(0+(1<<2)+(3<<4),tmp.m);
			tt[1].m=_mm_shuffle(1+(0<<2)+(3<<4),tmp.m);
			tt[2].m=_mm_shuffle(1+(3<<2)+(0<<4),tmp.m);
			c.bMax=_mm_min_ps(c.bMax.m,tt[axis].m);
		}
	}
}

template <class Output,class Rays,class Selector,class TContext>
void KDTree::TraverseSplitted(Rays &rays,const Selector &sel,const Output &out,TContext *tContext) const {
	if(Output::type==otPrimary) {
		typedef RaySelector<(Selector::size>4?Selector::size/4:Selector::size)> TSelector;
		TSelector tsel[4];

		for(int n=0;n<sel.Num();n++) {
			int seg=(sel[n]&(Selector::size-1))/TSelector::size;
			tsel[seg].Add(sel,n);
		}

		for(int s=0;s<4;s++)
			TraverseFast(rays,tsel[s],out,tContext);
		return;
	}

	enum { numSplits=4 };
	Vec3q selDirs[numSplits];
	for(int n=0;n<numSplits;n++) {
		uint id=(n*sel.Num())/numSplits+sel.Num()/(numSplits*4);
		selDirs[n]=rays.Dir(sel[id]);
	}

	Selector tsel[numSplits];

	for(int n=0;n<sel.Num();n++) {
		floatq dst=selDirs[0]|rays.Dir(sel[n]);
		uint target=0;

		for(int s=1;s<numSplits;s++) {
			floatq tmp=selDirs[s]|rays.Dir(sel[n]);
			int bits=CountMaskBits(_mm_movemask_ps((tmp>dst).m));
			if(bits>=2) { dst=tmp; target=s; }
		}

		tsel[target].Add(sel,n);
	}

	for(int s=0;s<numSplits;s++)
		TraverseFast(rays,tsel[s],out,tContext);
}

template <class Output,class Rays,class Selector>
void KDTree::TraverseOptimized(Rays &rays,const Selector &sel,const Output &out) const {
	if(!sel.Num()) return;

	Selector selectors[9];
	rays.GenSelectors(sel,selectors);

	for(int k=0;k<8;k++) {
		Selector &sel=selectors[k];
		if(sel.Num()) {
			// Ulepszyc, ta czworka moze byc popsuta (wektorki w roznych kierunkach)
			Vec3q dir0=rays.Dir(sel[0]);

			if(Output::type!=otPrimary) for(int i=1;i<sel.Num();i++) {
				int q=sel[i];

				int bitMask=sel.BitMask(i);
				if(CountMaskBits(bitMask)==1) { selectors[8].Add(q,bitMask); sel.Disable(i--); continue; }

				floatq dot=rays.Dir(q)|dir0;
				if(ForAny(dot<Const<floatq,998,1000>())) { selectors[8].Add(q); sel.Disable(i--); continue; }
			}
			TraverseFast(rays,sel,out,(TraverseContext<Rays,Selector>*)0);
		}
	}
	if(selectors[8].Num()) {
		const Selector &sel=selectors[8];
		TraverseMono(rays,sel,out);
	}
}

#endif

