#ifndef RTRACER_KDTRAVERSE_MONO_H
#define RTRACER_KDTRAVERSE_MONO_H

template <class Output,class Vec>
void KDTree::FullTraverse(const Vec &rOrigin,const Vec &rDir,const Output &out) const {	
	typedef typename Vec::TScalar real;
	typedef typename Output::integer integer;

	if(Output::objectIdsFlag)
		out.object[0]=Const<integer,0>();

	TreeStats stats;
//	stats.TracingRay();

	for(int n=0;n<objects.size();n++) {
		stats.Intersection();

		const Object &obj=objects[n];
		real dst=obj.Collide(rOrigin,rDir);
		typename Vec::TBool mask=dst>Const<real,0>()&&dst<out.dist[0];

		if(Output::objectIdsFlag) {
			integer test=mask;
			out.object[0]=Condition(test,integer(n),out.object[0]);
		}
		out.dist[0]=Condition(mask,dst,out);
	}

	out.stats->Update(stats);
}


template <class Output>
INLINE void KDTree::Traverse(const Vec3p &rOrigin,const Vec3p &tDir,const Output &out) const {
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
//	stats.TracingRay();

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

template <class Output,class Rays,class Selector>
void KDTree::TraverseMono(Rays &rays,const Selector &sel,const Output &out) const {
	Vec3p orig[4];

	if(Rays::sharedOrigin)
		Convert(rays.Origin(sel[0]),orig);
	
	for(int i=0;i<sel.Num();i++) {
		int q=sel[i];
		Vec3p dir[4];
		int msk=sel.BitMask(i);

		if(!Rays::sharedOrigin)
			Convert(rays.Origin(q),orig);
		Convert(rays.Dir(q),dir);

		float *dist=(float*)(out.dist+q); u32 tmp[4];
		u32 *objId=Output::objectIndexes?(u32*)(out.object+q):tmp;

		if(msk&1) Traverse(orig[0],dir[0],::Output<otNormal,float,u32>(dist+0,objId+0,out.stats));
		if(msk&2) Traverse(orig[1],dir[1],::Output<otNormal,float,u32>(dist+1,objId+1,out.stats));
		if(msk&4) Traverse(orig[2],dir[2],::Output<otNormal,float,u32>(dist+2,objId+2,out.stats));
		if(msk&8) Traverse(orig[3],dir[3],::Output<otNormal,float,u32>(dist+3,objId+3,out.stats));

	}
}

#endif

