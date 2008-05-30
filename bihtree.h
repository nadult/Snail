#ifndef RTRACER_BIHTREE_H
#define RTRACER_BIHTREE_H

#include "object.h"
#include "ray_group.h"
#include "tree_stats.h"
#include "context.h"

typedef TTriangle<SlowEdgeNormals> BIHTriangle;


// Generalnie:
// Lisc moze odwolywac sie maksymalnie do jednego obiektu
// Liscie bez obiektow nie sa przechowywane w drzewie, ojcowie ktorzy maja
// po jednym synu maja ustawiona flage OnlyOneChild
// Normalnie (jesli ojciec ma dwoch synow) to lewy syn jest na pozycji Child()+0
// a prawy na Child()+1

class BIHNode {
public:
	BIHNode(uint par) :parent(par) { }

	u32 Child() const { return val&0x3fffffff; }
	u32 Object() const { return val&0x3fffffff; }

	u32 Axis() const { return (val>>30); }
	bool IsLeaf() const { return (val>>30)==3; }
	float ClipLeft() const { return left; }
	float ClipRight() const { return right; }

	void SetLeaf(u32 object) { val=object|(3u<<30); }
	void SetNode(u32 axis,float l,float r,u32 child) { left=l; right=r; val=child|(axis<<30); }

	bool OnlyOneChild() const { return flags&1; }

	float left;
	float right;
	union {
		uint val;
		float fval;
	};
	union {
		uint parent;
		uint flags;
	};
};

template <class T>
class Vector
{
public:
	typedef T value_type;

	Vector() :tab(0),count(0) { }

	template <class Container>
	Vector(const Container &obj) :tab(0) {
		Alloc(obj.size());
		for(uint n=0;n<count;n++) tab[n]=obj[n];
	}
	Vector(const Vector &obj) :tab(0) {
		Alloc(obj.size());
		for(uint n=0;n<count;n++) tab[n]=obj[n];
	}
	const Vector &operator=(const Vector &obj) {
		if(&obj==this) return *this;
		return operator=<Vector>(obj);
	}
	template <class Container>
	const Vector &operator=(const Container &obj) {
		Alloc(obj.size());
		for(uint n=0;n<count;n++) tab[n]=obj[n];
		return *this;
	}
	~Vector() { Free(); }

	inline size_t size() const { return count; }
	inline const T &operator[](uint n) const { return tab[n]; }
	inline T &operator[](uint n) { return tab[n]; }

private:
	void Alloc(size_t newS) { Free(); count=newS; tab=count?new T[count]:0; }
	void Free() { if(tab) delete[] tab; } 

	T *tab;
	size_t count;
};

class BIHIdx {
public:
	BIHIdx() { }
	BIHIdx(uint i,const Vec3f &mi,const Vec3f &ma,float mul) :idx(i),min(mi),max(ma) {
		Vec3f vsize=max-min;
		size=Max(vsize.x,Max(vsize.y,vsize.z))*mul;
	}

	Vec3f min,max;
	float size;
	uint idx;
};


template <class TObject=BIHTriangle>
class BIHTree {
public:
	typedef TObject Object;
	enum { maxLevel=60 };

	BIHTree(const vector<Object> &obj);

	uint FindSimilarParent(uint nNode,uint axis) const;
	void PrintInfo(uint nNode=0,uint level=0) const;
	void Build(vector<BIHIdx> &indices,uint nNode,int first,int last,Vec3p min,Vec3p max,uint level);

	void FillDSignArray(int dirMask,int *dSign) const {
		dSign[0]=dirMask&1?1:0;
		dSign[1]=dirMask&2?1:0;
		dSign[2]=dirMask&4?1:0;
	}

	template <class Output>
	void TraverseMono(const Vec3p &rOrigin,const Vec3p &tDir,Output output) const {
		float maxD=output.dist[0];

		TreeStats stats;
		stats.TracingRay();

		Vec3p rDir=Vec3p(tDir.x+0.000000000001f,tDir.y+0.000000000001f,tDir.z+0.000000000001f);
		Vec3p invDir=VInv(rDir);

		int signMask=SignMask(floatq(invDir.m));
		int dSign[3]={signMask&1?1:0,signMask&2?1:0,signMask&4?1:0};
		float minRet=maxD,tMin=ConstEpsilon<float>(),tMax=maxD;

		struct Locals4 {
			float tMin,tMax;
			const BIHNode *node;
		} stackBegin[maxLevel+2],*stack=stackBegin;

		const BIHNode *node=&nodes[0],*node0=&nodes[0];
		tMax=Min(tMax,minRet);

		{
			Vec3p ttMin=(pMin-rOrigin)*invDir;
			Vec3p ttMax=(pMax-rOrigin)*invDir;
			if(dSign[0]) Swap(ttMin.x,ttMax.x);
			if(dSign[1]) Swap(ttMin.y,ttMax.y);
			if(dSign[2]) Swap(ttMin.z,ttMax.z);

			tMax=Min(Min(ttMax.x,ttMax.y),tMax);
			tMax=Min(ttMax.z,tMax);
			
			tMin=Max(Max(ttMin.x,ttMin.y),tMin);
			tMin=Max(ttMin.z,tMin);

			if(tMin>tMax) { stats.Skip(); goto EXIT; }
		}

		goto ENTRANCE;

		while(true) {
			if(stack==stackBegin) goto EXIT;
			stack--;
			tMin=stack->tMin;
			tMax=Min(stack->tMax,minRet);
			node=stack->node;
ENTRANCE:
			stats.LoopIteration();
			if(tMin>tMax) continue;

			u32 axis=node->Axis();
		
			if(axis==3) {
				uint objectId=node->Object();

				stats.Intersection();
				const Object &obj=objects[objectId];
				float ret=obj.Collide(rOrigin,tDir);
				if(ret<minRet&&ret>0) {
					minRet=ret;
					if(Output::objectIndexes)
						output.object[0]=objectId;

					tMax=Min(tMax,minRet);
					stats.IntersectPass();
				}
				else stats.IntersectFail();
				
				continue;
			}

			bool onlyOneChild=node->OnlyOneChild();
			uint sign=dSign[axis];
			float nearSplit,farSplit; {
				float start=(&rOrigin.x)[axis],inv=(&invDir.x)[axis];
				float near=node->ClipLeft(),far=node->ClipRight();
				if(sign) Swap(near,far);

				nearSplit=(near-start)*inv;
				farSplit =(far -start)*inv;
			}
			node=node0+node->Child();

			if(tMin>nearSplit) {
				if(tMax<farSplit) continue;

				tMin=Max(tMin,farSplit);
				if(!onlyOneChild) node+=(sign^1);
				goto ENTRANCE;
			}
			if(tMax<farSplit) {
				if(tMin>nearSplit) continue;

				tMax=Min(tMax,nearSplit);
				if(!onlyOneChild) node+=sign;
				goto ENTRANCE;
			}

			stack->tMin=Max(tMin,farSplit);
			stack->tMax=tMax;
			stack->node=node+(sign^1); stack++;

			tMax=Min(tMax,nearSplit);
			node+=sign;
			goto ENTRANCE;
		}

EXIT:
		output.stats->Update(stats);
		output.dist[0]=minRet;
		return;
	}

	template <class Output>
	void TraverseQuad(const Vec3q &rOrigin,const Vec3q &tDir,Output output,int dirMask) const {
		floatq maxD=output.dist[0];

		TreeStats stats;
		stats.TracingRay();

		Vec3q invDir=VInv(Vec3q(tDir.x+0.000000000001f,tDir.y+0.000000000001f,tDir.z+0.000000000001f));
		floatq tinv[3]={invDir.x,invDir.y,invDir.z};
		floatq torig[3]={rOrigin.x,rOrigin.y,rOrigin.z};

		int dSign[3]; FillDSignArray(dirMask,dSign);
		floatq minRet=maxD,tMin=ConstEpsilon<floatq>(),tMax=maxD;

		struct Locals4 {
			floatq tMin,tMax;
			const BIHNode *node;
		} stackBegin[maxLevel+2],*stack=stackBegin;

		const BIHNode *node=&nodes[0],*node0=&nodes[0];
		tMax=Min(tMax,minRet);

		{
			Vec3q ttMin=(Vec3q(pMin)-rOrigin)*invDir;
			Vec3q ttMax=(Vec3q(pMax)-rOrigin)*invDir;
			if(dSign[0]) Swap(ttMin.x,ttMax.x);
			if(dSign[1]) Swap(ttMin.y,ttMax.y);
			if(dSign[2]) Swap(ttMin.z,ttMax.z);

			tMax=Min(Min(ttMax.x,ttMax.y),tMax);
			tMax=Min(ttMax.z,tMax);
			
			tMin=Max(Max(ttMin.x,ttMin.y),tMin);
			tMin=Max(ttMin.z,tMin);
		}

		goto ENTRANCE;

		while(true) {
			if(stack==stackBegin) goto EXIT;

			stack--;
			tMin=stack->tMin;
			tMax=Min(stack->tMax,minRet);
			node=stack->node;
ENTRANCE:
			stats.LoopIteration();
			u32 axis=node->Axis();
		
			if(axis==3) {
				uint objectId=node->Object();

				stats.Intersection();
				const Object &obj=objects[objectId];
				floatq ret=obj.Collide(rOrigin,tDir);
				f32x4b mask=ret<minRet&&ret>0.0f;

				if(ForAny(mask)) {
					minRet=Condition(mask,ret,minRet);
					if(Output::objectIndexes)
						output.object[0]=Condition(i32x4b(mask),i32x4(objectId),output.object[0]);

					tMax=Min(tMax,minRet);
				}
				
				continue;
			}

			bool onlyOneChild=node->OnlyOneChild();
			uint sign=dSign[axis];
			floatq nearSplit,farSplit; {
				floatq start=torig[axis],inv=tinv[axis];
				float near=node->ClipLeft(),far=node->ClipRight();
				if(sign) Swap(near,far);

				nearSplit=(floatq(near)-start)*inv;
				farSplit =(floatq(far) -start)*inv;
			}
			node=node0+node->Child();

			f32x4b invalid=tMin>tMax;
			if(ForAll(tMin>nearSplit||invalid)) {
				if(ForAll(tMax<farSplit||invalid)) continue;

				tMin=Max(tMin,floatq(farSplit));
				if(!onlyOneChild) node+=(sign^1);
				goto ENTRANCE;
			}
			if(ForAll(tMax<farSplit||invalid)) {
				if(ForAll(tMin>nearSplit||invalid)) continue;

				tMax=Min(tMax,floatq(nearSplit));
				if(!onlyOneChild) node+=sign;
				goto ENTRANCE;
			}

			stack->tMin=Max(tMin,floatq(farSplit));
			stack->tMax=tMax;
			stack->node=node+(sign^1); stack++;

			tMax=Min(tMax,floatq(nearSplit));
			node+=sign;
			goto ENTRANCE;
		}

EXIT:
		output.stats->Update(stats);
		output.dist[0]=minRet;
		return;
	}

	void TraverseQuad4(const Vec3q *rOrigin,const Vec3q *tDir,floatq *out,i32x4 *object,TreeStats *tstats,int dirMask) const {
		enum { psize=4 };

		floatq maxD[4]={out[0],out[1],out[2],out[3]};

		TreeStats stats;
		stats.TracingRay();

		Vec3q invDir[4]={
			VInv(Vec3q(tDir[0].x+0.000000000001f,tDir[0].y+0.000000000001f,tDir[0].z+0.000000000001f)),
			VInv(Vec3q(tDir[1].x+0.000000000001f,tDir[1].y+0.000000000001f,tDir[1].z+0.000000000001f)),
			VInv(Vec3q(tDir[2].x+0.000000000001f,tDir[2].y+0.000000000001f,tDir[2].z+0.000000000001f)),
			VInv(Vec3q(tDir[3].x+0.000000000001f,tDir[3].y+0.000000000001f,tDir[3].z+0.000000000001f)) };
		floatq tinv[3][4]={
			{invDir[0].x,invDir[1].x,invDir[2].x,invDir[3].x},
			{invDir[0].y,invDir[1].y,invDir[2].y,invDir[3].y},
			{invDir[0].z,invDir[1].z,invDir[2].z,invDir[3].z} };
		floatq torig[3][4]={
			{rOrigin[0].x,rOrigin[1].x,rOrigin[2].x,rOrigin[3].x},
			{rOrigin[0].y,rOrigin[1].y,rOrigin[2].y,rOrigin[3].y},
			{rOrigin[0].z,rOrigin[1].z,rOrigin[2].z,rOrigin[3].z} };

		int dSign[3]; FillDSignArray(dirMask,dSign);
		floatq minRet[4]={maxD[0],maxD[1],maxD[2],maxD[3]},tMin[4],tMax[4];
		tMin[0]=tMin[1]=tMin[2]=tMin[3]=ConstEpsilon<floatq>();
		tMax[0]=Min(maxD[0],minRet[0]);
		tMax[1]=Min(maxD[1],minRet[1]);
		tMax[2]=Min(maxD[2],minRet[2]);
		tMax[3]=Min(maxD[3],minRet[3]);

		floatq fStackBegin[8*(maxLevel+2)],*fStack=fStackBegin;
		const BIHNode *nStackBegin[maxLevel+2],**nStack=nStackBegin;

		const BIHNode *node=&nodes[0],*node0=&nodes[0];

		{
			Vec3q ttMin[4]={
				(Vec3q(pMin)-rOrigin[0])*invDir[0],
				(Vec3q(pMin)-rOrigin[1])*invDir[1],
				(Vec3q(pMin)-rOrigin[2])*invDir[2],
				(Vec3q(pMin)-rOrigin[3])*invDir[3] };
			Vec3q ttMax[4]={
				(Vec3q(pMax)-rOrigin[0])*invDir[0],
				(Vec3q(pMax)-rOrigin[1])*invDir[1],
				(Vec3q(pMax)-rOrigin[2])*invDir[2],
				(Vec3q(pMax)-rOrigin[3])*invDir[3] };

			if(dSign[0]) {
				Swap(ttMin[0].x,ttMax[0].x);
				Swap(ttMin[1].x,ttMax[1].x);
				Swap(ttMin[2].x,ttMax[2].x);
				Swap(ttMin[3].x,ttMax[3].x); }
			if(dSign[1]) {
				Swap(ttMin[0].y,ttMax[0].y);
				Swap(ttMin[1].y,ttMax[1].y);
				Swap(ttMin[2].y,ttMax[2].y);
				Swap(ttMin[3].y,ttMax[3].y); }
			if(dSign[2]) {
				Swap(ttMin[0].z,ttMax[0].z);
				Swap(ttMin[1].z,ttMax[1].z);
				Swap(ttMin[2].z,ttMax[2].z);
				Swap(ttMin[3].z,ttMax[3].z); }

			tMax[0]=Min(Min(ttMax[0].x,ttMax[0].y),tMax[0]);
			tMax[1]=Min(Min(ttMax[1].x,ttMax[1].y),tMax[1]);
			tMax[2]=Min(Min(ttMax[2].x,ttMax[2].y),tMax[2]);
			tMax[3]=Min(Min(ttMax[3].x,ttMax[3].y),tMax[3]);

			tMax[0]=Min(ttMax[0].z,tMax[0]);
			tMax[1]=Min(ttMax[1].z,tMax[1]);
			tMax[2]=Min(ttMax[2].z,tMax[2]);
			tMax[3]=Min(ttMax[3].z,tMax[3]);
			
			tMin[0]=Max(Max(ttMin[0].x,ttMin[0].y),tMin[0]);
			tMin[1]=Max(Max(ttMin[1].x,ttMin[1].y),tMin[1]);
			tMin[2]=Max(Max(ttMin[2].x,ttMin[2].y),tMin[2]);
			tMin[3]=Max(Max(ttMin[3].x,ttMin[3].y),tMin[3]);

			tMin[0]=Max(ttMin[0].z,tMin[0]);
			tMin[1]=Max(ttMin[1].z,tMin[1]);
			tMin[2]=Max(ttMin[2].z,tMin[2]);
			tMin[3]=Max(ttMin[3].z,tMin[3]);
		}
		ObjectIdxBuffer<4> mailbox;

		goto ENTRANCE;

		while(true) {
			if(fStack==fStackBegin) goto EXIT;

			fStack-=8;
			tMin[0]=fStack[0];
			tMin[1]=fStack[1];
			tMin[2]=fStack[2];
			tMin[3]=fStack[3];
			tMax[0]=Min(fStack[4],minRet[0]);
			tMax[1]=Min(fStack[5],minRet[1]);
			tMax[2]=Min(fStack[6],minRet[2]);
			tMax[3]=Min(fStack[7],minRet[3]);
			--nStack; node=*nStack;
ENTRANCE:
			stats.LoopIteration();
			u32 axis=node->Axis();
		
			if(axis==3) {
				uint objectId=node->Object();
 
				if(mailbox.Find(objectId)) continue;

				const Object &obj=objects[objectId];
				floatq ret[4]; f32x4b mask[4];
				bool all=1;

				if(ForAny(tMin[0]<=tMax[0])) {				
					stats.Intersection();
					ret[0]=obj.Collide(rOrigin[0],tDir[0]);
					mask[0]=ret[0]<minRet[0]&&ret[0]>0.0f;
					minRet[0]=Condition(mask[0],ret[0],minRet[0]);
					object[0]=Condition(i32x4b(mask[0]),i32x4(objectId),object[0]);
					tMax[0]=Min(tMax[0],minRet[0]);
				}
				else all=0;

				if(ForAny(tMin[1]<=tMax[1])) {				
					stats.Intersection();
					ret[1]=obj.Collide(rOrigin[1],tDir[1]);
					mask[1]=ret[1]<minRet[1]&&ret[1]>0.0f;
					minRet[1]=Condition(mask[1],ret[1],minRet[1]);
					object[1]=Condition(i32x4b(mask[1]),i32x4(objectId),object[1]);
					tMax[1]=Min(tMax[1],minRet[1]);
				}
				else all=0;
			
				if(ForAny(tMin[2]<=tMax[2])) {				
					stats.Intersection();
					ret[2]=obj.Collide(rOrigin[2],tDir[2]);
					mask[2]=ret[2]<minRet[2]&&ret[2]>0.0f;
					minRet[2]=Condition(mask[2],ret[2],minRet[2]);
					object[2]=Condition(i32x4b(mask[2]),i32x4(objectId),object[2]);
					tMax[2]=Min(tMax[2],minRet[2]);
				}
				else all=0;

				if(ForAny(tMin[3]<=tMax[3])) {				
					stats.Intersection();
					ret[3]=obj.Collide(rOrigin[3],tDir[3]);
					mask[3]=ret[3]<minRet[3]&&ret[3]>0.0f;
					minRet[3]=Condition(mask[3],ret[3],minRet[3]);
					object[3]=Condition(i32x4b(mask[3]),i32x4(objectId),object[3]);
					tMax[3]=Min(tMax[3],minRet[3]);
				}
				else all=0;

				if(all) mailbox.Insert(objectId);
				
				continue;
			}

			bool onlyOneChild=node->OnlyOneChild();
			uint sign=dSign[axis];
			floatq near[4],far[4]; {
				floatq start[4]={torig[axis][0],torig[axis][1],torig[axis][2],torig[axis][3]};
				floatq inv[4]={tinv[axis][0],tinv[axis][1],tinv[axis][2],tinv[axis][3]};

				float tnear=node->ClipLeft(),tfar=node->ClipRight();
				if(sign) Swap(tnear,tfar);

				near[0]=Min( (floatq(tnear)-start[0])*inv[0], tMax[0]);
				near[1]=Min( (floatq(tnear)-start[1])*inv[1], tMax[1]);
				near[2]=Min( (floatq(tnear)-start[2])*inv[2], tMax[2]);
				near[3]=Min( (floatq(tnear)-start[3])*inv[3], tMax[3]);

				far [0]=Max( (floatq(tfar) -start[0])*inv[0], tMin[0]);
				far [1]=Max( (floatq(tfar) -start[1])*inv[1], tMin[1]);
				far [2]=Max( (floatq(tfar) -start[2])*inv[2], tMin[2]);
				far [3]=Max( (floatq(tfar) -start[3])*inv[3], tMin[3]);
			}
			node=node0+node->Child();

			f32x4b test1=tMin[0]>near[0]&&tMin[1]>near[1]&&tMin[2]>near[2]&&tMin[3]>near[3];
			f32x4b test2=tMax[0]<far [0]&&tMax[1]<far [1]&&tMax[2]<far [2]&&tMax[3]<far [3];

			if(ForAll(test1)) {
				if(ForAll(test2)) continue;

				tMin[0]=far[0];
				tMin[1]=far[1];
				tMin[2]=far[2];
				tMin[3]=far[3];
				if(!onlyOneChild) node+=(sign^1);
				goto ENTRANCE;
			}
			if(ForAll(test2)) {
				if(ForAll(test1)) continue;

				tMax[0]=near[0];
				tMax[1]=near[1];
				tMax[2]=near[2];
				tMax[3]=near[3];
				if(!onlyOneChild) node+=sign;
				goto ENTRANCE;
			}

			fStack[0]=far[0];
			fStack[1]=far[1];
			fStack[2]=far[2];
			fStack[3]=far[3];
			fStack[4]=tMax[0];
			fStack[5]=tMax[1];
			fStack[6]=tMax[2];
			fStack[7]=tMax[3];
			fStack+=8;

			*nStack=node+(sign^1);
			nStack++;

			tMax[0]=near[0];
			tMax[1]=near[1];
			tMax[2]=near[2];
			tMax[3]=near[3];
			node+=sign;
			goto ENTRANCE;
		}

EXIT:
		if(tstats) tstats->Update(stats);
		out[0]=minRet[0];
		out[1]=minRet[1];
		out[2]=minRet[2];
		out[3]=minRet[3];
	}

	template <class Output,class Group>
	void TraverseMonoGroup(Group &group,const RaySelector<Group::size> &sel,const Output &out) const {
		Vec3p orig[4],dir[4];
		u32 tmp[4];

		if(Group::sharedOrigin)
			Convert(group.Origin(sel[0]),orig);
		
		for(int i=0;i<sel.Num();i++) {
			int q=sel[i];
			Vec3p dir[4];
			int bmask=sel.BitMask(i);

			if(!Group::sharedOrigin)
				Convert(group.Origin(q),orig);
			Convert(group.Dir(q),dir);

			float *dist=(float*)(out.dist+q);
			u32 *objId=Output::objectIndexes?(u32*)(out.object+q):tmp;

			if(bmask&1) TraverseMono(orig[0],dir[0],::Output<otNormal,float,u32>(dist+0,objId+0,out.stats));
			if(bmask&2) TraverseMono(orig[1],dir[1],::Output<otNormal,float,u32>(dist+1,objId+1,out.stats));
			if(bmask&4) TraverseMono(orig[2],dir[2],::Output<otNormal,float,u32>(dist+2,objId+2,out.stats));
			if(bmask&8) TraverseMono(orig[3],dir[3],::Output<otNormal,float,u32>(dist+3,objId+3,out.stats));
		}
	}

	template <class Output,class Group>
	void TraverseQuadGroup(Group &group,const RaySelector<Group::size> &sel,const Output &out,int dirMask) const {
		i32x4 tmp;
		int i=0;
		for(;i+3<sel.Num();i+=4) {
			int q[4]={sel[i],sel[i+1],sel[i+2],sel[i+3]};
			Vec3q tOrig[4],tDir[4];
			floatq dist[4]; i32x4 obj[4];

			dist[0]=out.dist[q[0]]; dist[1]=out.dist[q[1]]; dist[2]=out.dist[q[2]]; dist[3]=out.dist[q[3]];
			obj[0]=out.object[q[0]]; obj[1]=out.object[q[1]]; obj[2]=out.object[q[2]]; obj[3]=out.object[q[3]];
			tOrig[0]=group.Origin(q[0]); tOrig[1]=group.Origin(q[1]);
			tOrig[2]=group.Origin(q[2]); tOrig[3]=group.Origin(q[3]);
			tDir[0]=group.Dir(q[0]); tDir[1]=group.Dir(q[1]);
			tDir[2]=group.Dir(q[2]); tDir[3]=group.Dir(q[3]);

			TraverseQuad4(tOrig,tDir,dist,obj,out.stats,dirMask);

			out.dist[q[0]]=dist[0]; out.dist[q[1]]=dist[1]; out.dist[q[2]]=dist[2]; out.dist[q[3]]=dist[3];
			out.object[q[0]]=obj[0]; out.object[q[1]]=obj[1]; out.object[q[2]]=obj[2]; out.object[q[3]]=obj[3];
		}
		for(;i<sel.Num();i++) {
			int q=sel[i];
			//int bmask=sel.BitMask(q);

			TraverseQuad(group.Origin(q),group.Dir(q),::Output<otNormal,f32x4,i32x4>(out.dist+q,out.object+q,out.stats),dirMask);
		}
	}


	template <class Output,class Group>
	void TraverseOptimized(Group &group,const RaySelector<Group::size> &sel,const Output &out) const {
		if(Output::type!=otPrimary) {
			TraverseMonoGroup(group,sel,out);
			return;
		}

		RaySelector<Group::size> selectors[9];
		group.GenSelectors(sel,selectors);

		for(int s=0;s<8;s++)
			TraverseQuadGroup(group,selectors[s],out,s);
		TraverseMonoGroup(group,selectors[8],out);
	}

	Vec3p pMin,pMax;
	vector<BIHNode> nodes;
	Vector<BIHTriangle> objects;
};

#include "bihtree.inl"

#endif

