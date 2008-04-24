#ifndef KDTRAVERSAL_H
#define KDTRAVERSAL_H

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


template <class Group,class Selector>
class TraverseContext {
public:
	TraverseContext(Group &group,const Selector &selector,const KDTree &tree,TraverseContext *other) {
		sel=selector;

		int xSign,ySign,zSign; {
			Vec3q &dir0=group.Dir(sel[0]);
			xSign=dir0.x[0]<0;
			ySign=dir0.y[0]<0;
			zSign=dir0.z[0]<0;
		}

		sign0[0]=xSign==0;
		sign0[1]=ySign==0;
		sign0[2]=zSign==0;

		negMaskI[0]=xSign?0x80000000:0;
		negMaskI[1]=ySign?0x80000000:0;
		negMaskI[2]=zSign?0x80000000:0;
		negMaskI[3]=0;

		{
			ComputeMinMaxOrigin(sel,group,minOR,maxOR);
			minOR=Neg(minOR); maxOR=Neg(maxOR);
			Vec3p tMin=VMin(minOR,maxOR);
			maxOR=VMax(minOR,maxOR); minOR=tMin;	
		}

		ComputePV(sel,&group.Dir(0),pv);
		ComputeOV(minOR,maxOR,pv,ov);

		stackPos=0;

		node=&tree.nodes[0];
		objIds=&tree.objectIds[0];
		objs=&tree.objects[0];

				{
			Vec3p tMin=Neg(tree.pMin),tMax=Neg(tree.pMax);
			Vec3p tpMin=VMin(tMin,tMax),tpMax=VMax(tMin,tMax);
			bMin=VMax(minOR,tpMin);
			bMax=tpMax;
		}

		if(sel.Num()>4)
			ComputeBeam(group,sel,Neg(minOR),Neg(maxOR),beamDir,beamOrig,beamM,beamA);

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
	}

	Vec3p Neg(const Vec3p &v) const { return Vec3p(_mm_xor_ps(v.m,negMask)); }

	RaySelector<Group::size> sel;

	int sign0[3];
	union { int negMaskI[4]; __m128 negMask; };

	Vec3p minOR,maxOR;
	floatq pv[3],ov[3];
	Vec3p beamDir,beamOrig; float beamM,beamA;
	
	Vec3p boxStack[(KDTree::MaxLevel+2)*2];
	const KDNode *nodeStack[KDTree::MaxLevel+2];
	u32 stackPos;

	const KDNode *node;
	const u32 *objIds;
	const KDTree::Object *objs;
	Vec3p bMin,bMax;

	ObjectIdxBuffer<8> idxBuffer;
};


#define NEG(vec) (c.Neg(vec))

template <class Output,class Group>
inline void KDTree::TraverseFast(Group &group,const RaySelector<Group::size> &tSelector,const Output &out,uint splittedYet,
		TraverseContext<Group,RaySelector<Group::size> > *tContext) const
{
	if(tSelector.Num()<3) {
		TraverseMonoGroup(group,tSelector,out);
		return;
	}

	if(!splittingFlag) splittedYet=1;

	TraverseContext<Group,RaySelector<Group::size> > c(group,tSelector,*this,tContext);

	TreeStats stats;
	stats.TracingPacket(c.sel.Num()*4);
	u32 axis;


	while(true) {
		stats.LoopIteration();

		axis=c.node->Axis();
		if(axis==3) { // Leaf
			if(c.node->NumObjects()) {

				const u32 *oid=c.objIds+c.node->FirstObject();
				
				Vec3p nodeMin,nodeMax; {
					Vec3p nodeEps(Const<floatq,1,1000>().m);
					Vec3p tMin=NEG(c.bMin),tMax=NEG(c.bMax);
					nodeMin=VMin(tMin,tMax)-nodeEps;
					nodeMax=VMax(tMin,tMax)+nodeEps;
				}

				float density=0.0f;
				for(int n=c.node->NumObjects();n>0;n--) {
					int tid=*oid++;
					const Object &obj=c.objs[tid];
					const bool fullInNode=obj.GetFlag2();
					
					if(c.idxBuffer.Find(tid)) continue;

					int beamCollision=1;
					if(c.sel.Num()>4) {
						beamCollision=obj.BeamCollide(c.beamOrig,c.beamDir,c.beamM,c.beamA);

					 	if(!beamCollision) { c.idxBuffer.Insert(tid); stats.Breaking(); continue; }

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

		//			if(beamCollision==2) {
		//				stats.NotBreaking();
		//			}
		//			else stats.Breaking();

					bool fullInside=1;
					floatq retMax=0.0f;

					for(int i=0;i<c.sel.Num();i++) {
						int q=c.sel[i];
						stats.Intersection();

						floatq ret=obj.Collide(group.Origin(q),group.Dir(q));
						f32x4b mask=(ret>Const<floatq,0>()&&ret<out.dist[q]);
						u32 msk=ForWhich(mask);
						retMax=Max(retMax,ret);


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
						c.sel.SetBitMask(q,c.sel.BitMask(q)&~msk);
					}
	
					{
						float r=Max(Max(retMax[0],retMax[1]),Max(retMax[2],retMax[3]));
						density=Max(density,r*obj.InvSize()*(0.0005f*Group::size));
					}	
					if(fullInside) c.idxBuffer.Insert(tid);
				}

				for(int i=0;i<c.sel.Num();i++)
					if(!c.sel.BitMask(c.sel[i]))
						c.sel.Disable(i--);
				if(c.sel.Num()==0) { out.stats->Update(stats); return; }
	
			if(out.density) *out.density=density;
			if(Group::recLevel>1&&density>1.0f&&splittedYet==0&&c.sel.Num()>Group::size*2/3) { // Too many triangles / ray, splitting
				stats.Skip();
				TraverseSplitted(group,c.sel,out,&c);
				out.stats->Update(stats);
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
		if(_mm_movemask_ps(_mm_cmplt_ps(c.bMax.m,split.m))&mask) { c.node+=c.sign0[axis]^1; continue; }

		{
			f32x4 abab=c.pv[axis]*split+c.ov[axis],tmp,tt[3];
			tmp=_mm_unpacklo_ps(split.m,abab.m);
			tt[0].m=_mm_shuffle(0+(1<<2)+(3<<4),tmp.m);
			tt[1].m=_mm_shuffle(1+(0<<2)+(3<<4),tmp.m);
			tt[2].m=_mm_shuffle(1+(3<<2)+(0<<4),tmp.m);
			c.boxStack[c.stackPos*2+0]=_mm_max_ps(c.bMin.m,tt[axis].m);

			c.nodeStack[c.stackPos]=c.node+c.sign0[axis];
			c.node+=c.sign0[axis]^1;
			c.boxStack[c.stackPos*2+1]=c.bMax;
			c.stackPos++;

			tmp=_mm_unpackhi_ps(split.m,abab.m);
			tt[0].m=_mm_shuffle(0+(1<<2)+(3<<4),tmp.m);
			tt[1].m=_mm_shuffle(1+(0<<2)+(3<<4),tmp.m);
			tt[2].m=_mm_shuffle(1+(3<<2)+(0<<4),tmp.m);
			c.bMax=_mm_min_ps(c.bMax.m,tt[axis].m);
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

template <class Output,class Group,class Selector>
void KDTree::TraverseSplitted(Group &group,const Selector &sel,const Output &out,
							TraverseContext<Group,Selector> *tContext) const {
	bool primary=1;

	if(primary) {
		RaySelector<Group::size> tsel[4];
		for(int n=0;n<sel.Num();n++) {
			tsel[sel[n]/(Group::size/4)].Add(sel[n],sel.BitMask(sel[n]));
		}
		for(int s=0;s<4;s++)
			TraverseFast(group,tsel[s],out,1,tContext);
		return;
	}

	enum { numSplits=4 };
	Vec3q selDirs[numSplits];
	for(int n=0;n<numSplits;n++) {
		uint id=(n*sel.Num())/numSplits+sel.Num()/(numSplits*4);
		selDirs[n]=group.Dir(sel[id]);
	}

	RaySelector<Group::size> tsel[numSplits];

	for(int n=0;n<sel.Num();n++) {
		floatq dst=selDirs[0]|group.Dir(sel[n]);
		uint target=0;

		for(int s=1;s<numSplits;s++) {
			floatq tmp=selDirs[s]|group.Dir(sel[n]);
			int bits=CountMaskBits(_mm_movemask_ps((tmp>dst).m));
			if(bits>=2) { dst=tmp; target=s; }
		}

		tsel[target].Add(sel[n],sel.BitMask(sel[n]));
	}

	for(int s=0;s<numSplits;s++)
		TraverseFast(group,tsel[s],out,1,tContext);
}

template <class Output,class Group,class Selector>
void KDTree::TraverseOptimized(Group &group,const Selector &sel,const Output &out,bool primary) const {
	if(!sel.Num()) return;

	RaySelector<Group::size> selectors[9];
	group.GenSelectors(sel,selectors);

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
			TraverseFast(group,sel,out);
		}
	}
	if(selectors[8].Num()) {
		const RaySelector<Group::size> &sel=selectors[8];
		TraverseMonoGroup(group,sel,out);
	}
}

#endif

