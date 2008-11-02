#ifndef RTRACER_BVH_H
#define RTRACER_BVH_H

#include "rtbase.h"
#include "sphere.h"
#include "triangle.h"
#include "context.h"
#include "ray_group.h"
#include "base_scene.h"

class BIHTree;

class Object: public RefCounter {
public:
	virtual ~Object() { }
	virtual void TraverseMono(const Vec3p &rOrigin,const Vec3p &rDir,Output<otNormal,float,u32> output,int) const=0;
	virtual void TraverseQuad(const Vec3q &rOrigin,const Vec3q &rDir,Output<otNormal,f32x4,i32x4> output,int) const=0;
	virtual void TraverseQuad4(const Vec3q *rOrigin,const Vec3q *tDir,Output<otNormal,f32x4,i32x4> output,int) const=0;
	virtual BBox GetBBox() const=0;

	// number of triangles for example
	virtual int Complexity() const = 0;

	template <class Derived>
	Vec3f TFlatNormals(u32 elementId) const {
		typedef typename Derived::Element Element;
		const Derived &derived=*(Derived*)this;

		// transform by matrix ?
		return derived.objects[elementId].Nrm();
		
/*		Vec nrm(e0->Nrm());

		if(ForAny(integer(elementId[0])!=elementId)) for(int n=1;n<ScalarInfo<real>::multiplicity;n++) {
			const Element *eN=&derived.objects[elementId[n]];
			if(eN!=e0) {
				Vec newNrm(eN->Nrm());
				nrm=Condition(ScalarInfo<real>::ElementMask(n),newNrm,nrm);
			}
		}

		return nrm*RSqrt(nrm|nrm);*/
	}
	
	virtual Vec3f FlatNormals(u32) const { return Vec3f(0,1,0); }
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
	void Traverse(const Rays &rays,const RaySelector<Rays::size> &sel,Output<otNormal,f32x4,i32x4> output,int nNode=0) const {
		const Node &node=nodes[nNode];
		//output.stats->Skip();
		
		enum { shared=1 };
		
		if(node.count) {
			if(node.count==2) {
				const Node &a=nodes[node.subNode+0],&b=nodes[node.subNode+1];
				bool left=0,right=0;

				for(int r=0;r<Rays::size&&!left;r++)
					left|=a.bBox.Test(rays.Origin(shared?0:r),rays.Dir(r),output.dist[r]);
				for(int r=0;r<Rays::size&&!right;r++)
					right|=b.bBox.Test(rays.Origin(shared?0:r),rays.Dir(r),output.dist[r]);
				
				if(left&&right) {
					Vec3f dir; {
						Vec3q dirq=rays.Dir(0);
						dir=Vec3f(dirq.x[0],dirq.y[0],dirq.z[0]);
					}
					
					if((&dir.x)[node.divAxis]>0) {
						Traverse(rays,sel,output,node.subNode+0);
						Traverse(rays,sel,output,node.subNode+1);
					}
					else {
						Traverse(rays,sel,output,node.subNode+1);
						Traverse(rays,sel,output,node.subNode+0);
					}
				}
				else {
					if(left) Traverse(rays,sel,output,node.subNode+0);
					if(right) Traverse(rays,sel,output,node.subNode+1);
				}
			}
			else for(int n=0;n<node.count;n++) {
				const Node &tNode=nodes[node.subNode+n];
				
				bool test=0;
				for(int r=0;r<Rays::size&&!test;r++)
					test|=tNode.bBox.Test(rays.Origin(shared?0:r),rays.Dir(r),output.dist[r]);
					
				if(test) Traverse(rays,sel,output,node.subNode+n);
			}
		}
		else {
			
			bool test=1;
			
			if(gVals[0]) {
				test=0; for(int r=0;r<Rays::size&&!test;r++)
					test|=node.optBBox.Test(rays.Origin(shared?0:r),rays.Dir(r),output.dist[r]);
			} else test=1;
			
			if(test) {
				output.stats->Skip();
				
				Vec3q orig[Rays::size],dir[Rays::size];
				TransformRays<Rays,shared>(rays,orig,dir,node.invTrans);
			
				int n=0;
				for(;n<Rays::size-3;n+=4)
					objects[node.subNode]->TraverseQuad4(orig+(shared?0:n),dir+n,Output<otNormal,f32x4,i32x4>(output,n),node.id);
				for(;n<Rays::size;n++)
					objects[node.subNode]->TraverseQuad(orig[shared?0:n],dir[n],Output<otNormal,f32x4,i32x4>(output,n),node.id);
			}
		}
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
		OptBBox optBBox;
		
		int subNode,count,id,divAxis;
	};
	
	vector<PObject> objects;
	vector<Node,AlignedAllocator<Node> > nodes;
};


#endif
