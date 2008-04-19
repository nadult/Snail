#ifndef RTRACER_BIHTREE_H
#define RTRACER_BIHTREE_H

#include "object.h"
#include "ray_group.h"
#include "tree_stats.h"


class BIHNode {
public:
	BIHNode(uint par) :val(0),parent(par) { }

	u32 Child() const { return val&0x3fffffff; }
	u32 FirstObject() const { return val&0x3fffffff; }
	u32 NumObjects() const { return num; }

	u32 Axis() const { return (val>>30); }
	bool Leaf() const { return (val>>30)==3; }
	float ClipLeft() const { return left; }
	float ClipRight() const { return right; }

	void SetLeaf(u32 first,u32 n) { num=n; val=first|(3u<<30); }
	void SetNode(u32 axis,float l,float r,u32 nextAdv) { left=l; right=r; val=nextAdv|(axis<<30); }

	union {
		float left;
		uint num;
	};
	float right;
	uint val;
	uint parent;
};

class BIHTree {
public:
	enum { maxLevel=80 };

	BIHTree(const vector<Object> &obj) :objects(obj) {
		if(!objects.size()) return;

		pMin=objects[0].BoundMin();
		pMax=objects[0].BoundMax();

		for(uint n=1;n<objects.size();n++) {
			pMin=VMin(pMin,objects[n].BoundMin());
			pMax=VMax(pMax,objects[n].BoundMax());
		}
		nodes.push_back(BIHNode(0));
		
		Build(0,0,objects.size()-1,pMin,pMax,0);
	}

	uint FindParent(uint nNode,uint child,uint axis) const {
		const BIHNode &node=nodes[nNode];

		if(node.Child()!=child)
			if(nodes[node.Child()].Axis()!=3||nodes[node.Child()].NumObjects()>0)
				return ~0;
		if(node.Child()+1!=child)
			if(nodes[node.Child()+1].Axis()!=3||nodes[node.Child()+1].NumObjects()>0)
				return ~0;
		
		if(axis==node.Axis())
			return nNode;

		if(nNode==0) return ~0;
		FindParent(node.parent,nNode,axis);
	}

	void PrintInfo(uint nNode=0,uint level=0) const {
		if(nNode==0) {
			printf("Objects %d    Nodes: %d\n",objects.size(),nodes.size());

			double objSize=0;

			for(int n=0;n<objects.size();n++) {
				const Object &o=objects[n];
				Vec3p bMin=o.BoundMin(),bMax=o.BoundMax();
				objSize+=bMax.x+bMax.y+bMax.z-bMin.x-bMin.y-bMin.z;
			//	printf("%d box: (%f %f %f) (%f %f %f)\n",n,bMin.x,bMin.y,bMin.z,bMax.x,bMax.y,bMax.z);
			}

			printf("Box: (%f %f %f) (%f %f %f)\n",pMin.x,pMin.y,pMin.z,pMax.x,pMax.y,pMax.z);
			printf("Average object size: %.2f\n",objSize/double(6*objects.size()));
			return;
		}
		for(int n=0;n<level;n++) printf(" ");
		const BIHNode &node=nodes[nNode];
		if(node.Axis()==3) { printf("Leaf: %d objects\n",node.NumObjects()); }
		else {
			printf("Node: %d %f %f\n",node.Axis(),node.ClipLeft(),node.ClipRight());
			PrintInfo(node.Child()+0,level+1);
			PrintInfo(node.Child()+1,level+1);
		}
	}

	void Build(uint nNode,int first,int last,Vec3p min,Vec3p max,uint level) {
	//	for(int n=0;n<level;n++) printf(" ");
	//	printf("(%.2f %.2f %.2f) %d\n",max.x-min.x,max.y-min.y,max.z-min.z,last-first+1);

		if(last-first+1<=1||level>=maxLevel) {
			BIHNode &node=nodes[nNode];
			node.SetLeaf(first,last-first+1);
			return;
		}

		uint axis; {
			axis=0;
			float size=max.x-min.x;
			if(max.y-min.y>size) { size=max.y-min.y; axis=1; }
			if(max.z-min.z>size) axis=2;
		}
		
		float split=Lerp((&min.x)[axis],(&max.x)[axis],0.5f);
		float leftMax=(&pMin.x)[axis],rightMin=(&pMax.x)[axis];
		int right=last;

		for(int n=first;n<=right;n++) {
			float pos,min,max; {
				min=(&objects[n].BoundMin().x)[axis];
				max=(&objects[n].BoundMax().x)[axis];
				pos=Lerp(min,max,0.5f);
			}
			if(pos>=split) {
				Swap(objects[n--],objects[right--]);
				rightMin=Min(rightMin,min);
			}
			else {
				leftMax=Max(leftMax,max);
			}
		}

		int numLeft=right-first+1;	
		int numRight=last-right;

		Vec3p maxL=max; (&maxL.x)[axis]=split;
		Vec3p minR=min; (&minR.x)[axis]=split;

		if((numLeft==0||numRight==0)&&nNode!=0) {
			uint sameAxisParent=FindParent(nodes[nNode].parent,nNode,axis);
			if(sameAxisParent!=~0) {
				if(numLeft==0) Build(nNode,first,last,minR,max,level+1);
				if(numRight==0) Build(nNode,first,last,min,maxL,level+1);
				return;
			}
		}

		{
			BIHNode &node=nodes[nNode];
			node.SetNode(axis,leftMax,rightMin,nodes.size());
		}
		nodes.push_back(BIHNode(nNode)); nodes.back().SetLeaf(first,numLeft);
		nodes.push_back(BIHNode(nNode)); nodes.back().SetLeaf(right+1,numRight);
		uint cLeft=nodes.size()-2,cRight=nodes.size()-1;

		Build(cLeft,first,right,min,maxL,level+1);
		Build(cRight,right+1,last,minR,max,level+1);
	}

	void TraverseMono(const Vec3p &rOrigin,const Vec3p &tDir,const float &maxD,float &dist,u32 *outObj) const {
		float min=maxD+10;
		*outObj=0;

		Vec3p rDir=Vec3p(tDir.x+0.000000000001f,tDir.y+0.000000000001f,tDir.z+0.000000000001f);
		Vec3p invDir=VInv(rDir);

		int signMask=SignMask(floatq(invDir.m));
		int dSign[3]={signMask&1?1:0,signMask&2?1:0,signMask&4?1:0};
		float minRet=maxD,tMin=ConstEpsilon<float>(),tMax=maxD;

		struct Locals4 {
			float tMin,tMax;
			const BIHNode *node;
		} stackBegin[maxLevel+2],*stack=stackBegin;

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
		}

		const BIHNode *node=&nodes[0];
		TreeStats localStats;

#define POP_STACK() { \
				if(stack==stackBegin) { stats.Update(localStats); dist=minRet; return; } \
				{ stack--; tMin=stack->tMin; tMax=stack->tMax; node=stack->node; } \
				continue; }

		while(true) {
			localStats.iters++;
			u32 axis=node->Axis();
		
			if(tMin>minRet) {
				if(stack==stackBegin) { stats.Update(localStats); dist=minRet; return; }
				{ stack--; tMin=stack->tMin; tMax=stack->tMax; node=stack->node; }
				continue;
			}

			if(axis==3) {
				uint first=node->FirstObject();
				uint num=node->NumObjects();

				for(uint n=0;n<num;n++) {
					localStats.colTests++;
					const Object &obj=objects[n+first];
					float ret=obj.Collide(rOrigin,tDir);
					if(ret<minRet&&ret>0) {
						minRet=ret;
						*outObj=n+first;
					}	
				}
				
				POP_STACK();
			}

			uint sign=dSign[axis];
			float nearSplit,farSplit; {
				float start=(&rOrigin.x)[axis],inv=(&invDir.x)[axis];
				float near=node->ClipLeft(),far=node->ClipRight();
				if(sign) Swap(near,far);

				nearSplit=(near-start)*inv;
				farSplit =(far -start)*inv;
			}
			node=&nodes[node->Child()];

			if(tMin>nearSplit) {
				if(tMax<farSplit) POP_STACK();

				tMin=Max(tMin,farSplit);
				node+=(sign^1);
				continue;
			}
			if(tMax<farSplit) {
				if(tMin>nearSplit) POP_STACK();

				tMax=Min(tMax,nearSplit);
				node+=sign;
				continue;
			}

			stack->tMin=Max(tMin,farSplit);
			stack->tMax=tMax;
			stack->node=node+(sign^1); stack++;

			tMax=Min(tMax,nearSplit);
			node=node+sign;
		}
	}

	template <class Output,class Group>
	void TraverseMonoGroup(Group &group,const RaySelector<Group::size> &sel,const floatq &maxD,const Output &out) const {
		Vec3p orig[4],dir[4];
		float fmaxD[4];
		u32 tmp[4];

		if(Group::singleOrigin)
			Convert(group.Origin(sel[0]),orig);
		Convert(maxD,fmaxD);
		
		for(int i=0;i<sel.Num();i++) {
			int q=sel[i];
			Vec3p dir[4];

			if(!Group::singleOrigin)
				Convert(group.Origin(q),orig);
			Convert(group.Dir(q),dir);

			float *dist=(float*)(out.dist+q);
			u32 *objId=Output::objectIdsFlag?(u32*)(out.object+q):tmp;
			TraverseMono(orig[0],dir[0],fmaxD[0],dist[0],objId+0);
			TraverseMono(orig[1],dir[1],fmaxD[1],dist[1],objId+1);
			TraverseMono(orig[2],dir[2],fmaxD[2],dist[2],objId+2);
			TraverseMono(orig[3],dir[3],fmaxD[3],dist[3],objId+3);
		}
	//	stats.runs+=sel.Num()*4;
		stats.tracedRays+=sel.Num()*4;
	}


	mutable TreeStats stats;
	Vec3p pMin,pMax;
	vector<BIHNode> nodes;
	vector<Object> objects;
};

#endif

