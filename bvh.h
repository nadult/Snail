#ifndef RTRACER_BVH_H
#define RTRACER_BVH_H

#include "rtbase.h"
#include "sphere.h"
#include "triangle.h"
#include "context.h"
#include "ray_group.h"
#include "base_scene.h"

class Object: public RefCounter {
public:
	virtual ~Object() { }
	virtual void TraverseMono(const Vec3p &rOrigin,const Vec3p &rDir,Output<otNormal,float,u32> output,int) const=0;
	virtual void TraversePacket1(const RayGroup<1,0,1> &rays,const RaySelector<1> &selector,Output<otNormal,f32x4,i32x4> output,int) const=0;
	virtual void TraversePacket4(const RayGroup<4,0,1> &rays,const RaySelector<4> &selector,Output<otNormal,f32x4,i32x4> output,int) const=0;
	virtual void TraversePacket16(const RayGroup<16,0,1> &rays,const RaySelector<16> &selector,Output<otNormal,f32x4,i32x4> output,int) const=0;
	virtual BBox GetBBox() const=0;

	// number of triangles for example
	virtual int Complexity() const = 0;
	virtual Vec3f FlatNormals(u32,u32) const=0;
};

typedef Ptr<Object> PObject;

class BVHBuilder {
public:
	int AddObject(PObject object,const Matrix<Vec4f> &preTrans,const BBox &box,const OptBBox &optBBox);
	int AddInstance(int objId,const Matrix<Vec4f> &trans);

	struct Obj {
		PObject object;
		Matrix<Vec4f> preTrans;
		OptBBox optBBox;
		BBox bBox;
		u32 id;
	};
	
	struct Instance {
		Matrix<Vec4f> trans;
		u32 objId,id;
	};
	
	vector<Obj> objects;
	vector<Instance> instances;
};

class BVH {
public:
	void Build(const BVHBuilder&);

	BBox GetBBox() const;

protected:	
	template <class Rays,bool shared>
	static void TransformRays(const Rays &rays,Vec3q * __restrict__ orig,Vec3q *__restrict__ dir,const Matrix<Vec4f> &mat) {
		if(shared) orig[0]=mat*rays.Origin(0);
		
		for(int n=0;n<Rays::size;n++) {
			if(!shared) orig[n]=mat*rays.Origin(n);
			dir[n]=mat&rays.Dir(n);
		}
	}

public:

	template <class Rays>
	void TraversePacket(const Rays &rays,const RaySelector<Rays::size> &sel,Output<otNormal,f32x4,i32x4> output,int nNode=0) const {
		const Node &node=nodes[nNode];
		//output.stats->Skip();
		
		enum { shared=1 };
		int stack[maxDepth+2],stackPos;
		stack[stackPos++]=nNode;
		TreeStats stats;
		
		while(stackPos) {
			nNode=stack[--stackPos];
			const Node &node=nodes[nNode];
			stats.LoopIteration();

			if(node.count) {
				if(node.count==2) {
					const Node &a=nodes[node.subNode+0],&b=nodes[node.subNode+1];
					bool left=0,right=0;

					if(shared) left=a.bBox.TestIP<Rays::size>(rays.Origin(0),rays.idir,output.dist);
					else for(int r=0;r<Rays::size&&!left;r++)
						left|=a.bBox.TestI(rays.Origin(r),rays.IDir(r),output.dist[r]);

					if(shared) right=b.bBox.TestIP<Rays::size>(rays.Origin(0),rays.idir,output.dist);
					else for(int r=0;r<Rays::size&&!right;r++)
						right|=b.bBox.TestI(rays.Origin(r),rays.IDir(r),output.dist[r]);
					
					if(left&&right) {
						Vec3f dir; {
							Vec3q dirq=rays.Dir(Min(Rays::size-1,Rays::size/2+1));
							dir=Vec3f(dirq.x[1],dirq.y[1],dirq.z[1]);
						}
						
						if((&dir.x)[node.divAxis]>0) {
							stack[stackPos++]=node.subNode+1;
							stack[stackPos++]=node.subNode+0;
						}
						else {
							stack[stackPos++]=node.subNode+0;
							stack[stackPos++]=node.subNode+1;
						}
					}
					else {
						if(right) stack[stackPos++]=node.subNode+1;
						if(left) stack[stackPos++]=node.subNode+0;
					}
				}
				else for(int n=0;n<node.count;n++) {
					const Node &tNode=nodes[node.subNode+n];
					
					bool test=0;
					for(int r=0;r<Rays::size&&!test;r++)
						test|=tNode.bBox.TestI(rays.Origin(shared?0:r),rays.IDir(r),output.dist[r]);
						
					if(test) stack[stackPos++]=node.subNode+n;
				}
			}
			else {
				Vec3q orig[Rays::size],dir[Rays::size],inv[Rays::size];
				TransformRays<Rays,shared>(rays,orig,dir,node.invTrans);
				for(int q=0;q<Rays::size;q++) inv[q]=VInv(dir[q]);

				stats.Intersection();
				
				RaySelector<1> sel0;
				RaySelector<4> sel;
				RaySelector<16> sel1;
				sel.SelectAll(); sel1.SelectAll(); sel0.SelectAll();
				
				int n=0;
				for(;n<Rays::size-15;n+=16)
					objects[node.subNode]->TraversePacket16(RayGroup<16,0,1>(orig+(shared?0:n),dir+n,inv+n),sel1,
															Output<otNormal,f32x4,i32x4>(output,n),node.id);
				for(;n<Rays::size-3;n+=4)
					objects[node.subNode]->TraversePacket4(RayGroup<4,0,1>(orig+(shared?0:n),dir+n,inv+n),sel,
															Output<otNormal,f32x4,i32x4>(output,n),node.id);
				for(;n<Rays::size;n++)
					objects[node.subNode]->TraversePacket1(RayGroup<1,0,1>(orig+(shared?0:n),dir+n,inv+n),sel0,
															Output<otNormal,f32x4,i32x4>(output,n),node.id);
			}
		}

		if(output.stats) output.stats->Update(stats);
	}

		
protected:
	void FindSplit(int nNode,BBox sceneBox,const BVHBuilder&,vector<int> &indices,int first,int count,const vector<BBox> &instBBoxes,
					const vector<float> centers[3],int depth);
	
	void Update();
	void UpdateBU(int node,int nParent);
	
	int AddNode(int sub,int count);
	int maxDepth;
	
public:
	struct Node {
		Node() { divAxis=0; }
		template <class Real,class Vec>
		bool Test(Vec &rOrig,Vec &rDir,const Real &maxDist=Real(1.0f/0.0f)) const NOINLINE;
		
		INLINE bool IsLeaf() const { return count==0; }

		// used only in leafs
		Matrix<Vec4f> trans,invTrans;

		BBox bBox;
		
		int subNode,count,id,divAxis;
	};
	
	vector<PObject> objects;
	vector<Node,AlignedAllocator<Node> > nodes;
};


#endif
