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
	BIHIdx(uint i,const Vec3f &mi,const Vec3f &ma) :idx(i),min(mi),max(ma) {
		Vec3f vsize=max-min;
		size=Max(vsize.x,Max(vsize.y,vsize.z));
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

	void TraverseQuad4(const Vec3q *rOrigin,const Vec3q *tDir,floatq *out,i32x4 *object,TreeStats *stats,int dirMask) const {
		TraverseQuad(rOrigin[0],tDir[0],::Output<otNormal,f32x4,i32x4>(out+0,object+0,stats),dirMask);
		TraverseQuad(rOrigin[1],tDir[1],::Output<otNormal,f32x4,i32x4>(out+1,object+1,stats),dirMask);
		TraverseQuad(rOrigin[2],tDir[2],::Output<otNormal,f32x4,i32x4>(out+2,object+2,stats),dirMask);
		TraverseQuad(rOrigin[3],tDir[3],::Output<otNormal,f32x4,i32x4>(out+3,object+3,stats),dirMask);
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
		int i;
		for(i=0;i+3<sel.Num();i+=4) {
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

