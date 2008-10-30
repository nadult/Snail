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
	float BoxPointDistanceSq(const BBox &box,const Vec3f &point) const {
		Vec3f center=box.Center(),size=box.Size()*0.5f;
		float diff[3]={point.x-center.x,point.y-center.y,point.z-center.z};
		float length[3]={size.x,size.y,size.z};
		float p[3]={point.x,point.y,point.z};

		float distance = 0.0;
		float delta;

		for( int i=0; i<3; i++ )
			if ( diff[i]<-length[i] ) {
				delta=diff[i]+length[i];
				distance+=delta*delta;
				diff[i]=-length[i];
			}
			else if (diff[i]>length[i] ) {
				delta=diff[i]-length[i];
				distance+=delta*delta;
				diff[i]=length[i];
			}

		//if(where) *where=diff;
		return distance;
	}
	
public:
	template <class Rays>
	void Traverse(const Rays &rays,const RaySelector<Rays::size> &sel,Output<otNormal,f32x4,i32x4> output,int nNode=0) const {
		const Node &node=nodes[nNode];
		output.stats->Skip();
		
		if(node.count) {
			if(node.count==2&&gVals[0]) {
				const Node &a=nodes[node.subNode+0],&b=nodes[node.subNode+1];
			
				Vec3q orig0[Rays::size],orig1[Rays::size];
				Vec3q dir0 [Rays::size],dir1 [Rays::size];
				
				for(int n=0;n<Rays::size;n++) {
					orig0[n]=a.invTrans&rays.Origin(n);
					dir0[n]=a.invTrans&rays.Dir(n);
				}
				for(int n=0;n<Rays::size;n++) {
					orig1[n]=b.invTrans&rays.Origin(n);
					dir1[n]=b.invTrans&rays.Dir(n);
				}
		
				bool left=0,right=0;
				for(int n=0;n<Rays::size;n++)
					left|=a.Test(orig0[n],dir0[n],output.dist[n]);
				for(int n=0;n<Rays::size;n++)
					right|=b.Test(orig1[n],dir1[n],output.dist[n]);
				
				if(left&&right) {
					Vec3f size0=a.bBox.Size(),size1=b.bBox.Size();
					//Vec3f tDir(dir0[0].x[0],dir0[0].y[0],dir0[0].z[0]);
					
					float surf0=size0.x*(size0.y+size0.z)+size0.y*size0.z;
					float surf1=size1.x*(size1.y+size1.z)+size1.y*size1.z;
					
					
					if(gVals[2]) {
						if(gVals[1]) {
							surf0*=FastRSqrt(BoxPointDistanceSq(a.bBox,Vec3f(orig0[0].x[0],orig0[0].y[0],orig0[0].z[0])));
							surf1*=FastRSqrt(BoxPointDistanceSq(b.bBox,Vec3f(orig1[0].x[0],orig1[0].y[0],orig1[0].z[0])));
						} else {
							surf0*=FastRSqrt(LengthSq(a.bBox.Center()-Vec3f(orig0[0].x[0],orig0[0].y[0],orig0[0].z[0])));
							surf1*=FastRSqrt(LengthSq(b.bBox.Center()-Vec3f(orig1[0].x[0],orig1[0].y[0],orig1[0].z[0])));
						}
					}
					bool first0=surf0>surf1;
					
					/*int min[3],max[3];
					min[0]=b.bBox.min.x<a.bBox.min.x;
					min[1]=b.bBox.min.y<a.bBox.min.y;
					min[2]=b.bBox.min.z<a.bBox.min.z;
					max[0]=b.bBox.max.x>a.bBox.max.x;
					max[1]=b.bBox.max.y>a.bBox.max.y;
					max[2]=b.bBox.max.z>a.bBox.max.z;
					
					int maxAxis=Abs(tDir.x)>Abs(tDir.y)?0:1;
					if(Abs(tDir.z)>Abs((&tDir.x)[maxAxis])) maxAxis=2;
					bool first0=((&tDir.x)[maxAxis]<0?max:min)[maxAxis]==0; */
				
				/*	float left=
						Abs(tDir.x)*((tDir.x<0.0f?max:min)[0]==0)+
						Abs(tDir.y)*((tDir.y<0.0f?max:min)[1]==0)+
						Abs(tDir.z)*((tDir.z<0.0f?max:min)[2]==0);
					float right=
						Abs(tDir.x)*((tDir.x<0.0f?max:min)[0])+
						Abs(tDir.y)*((tDir.y<0.0f?max:min)[1])+
						Abs(tDir.z)*((tDir.z<0.0f?max:min)[2]);
					bool first0=left<right;*/
					
					if(first0) {
						Traverse(Rays(dir0,orig0),sel,output,node.subNode+0);
						Traverse(Rays(dir1,orig1),sel,output,node.subNode+1);
					}
					else {
						Traverse(Rays(dir1,orig1),sel,output,node.subNode+1);
						Traverse(Rays(dir0,orig0),sel,output,node.subNode+0);
					}
				}
				else {
					if(left) Traverse(Rays(dir0,orig0),sel,output,node.subNode+0);
					if(right) Traverse(Rays(dir1,orig1),sel,output,node.subNode+1);
				}
			}
			else for(int n=0;n<node.count;n++) {
				const Node &tNode=nodes[node.subNode+n];
				
				Vec3q orig[Rays::size],dir[Rays::size];
				for(int r=0;r<Rays::size;r++) {
					orig[r]=tNode.invTrans&rays.Origin(r);
					dir[r]=tNode.invTrans&rays.Dir(r);
				}
				
				bool test=0;
				for(int r=0;r<Rays::size;r++)
					test|=tNode.Test(orig[r],dir[r],output.dist[r]);
					
				if(test) Traverse(Rays(dir,orig),sel,output,node.subNode+n);
			}
		}
		else {
			int n=0;
			for(;n<Rays::size-3;n+=4)
				objects[node.subNode]->TraverseQuad4(&rays.Origin(n),&rays.Dir(n),Output<otNormal,f32x4,i32x4>(output,n),node.id);
			for(;n<Rays::size;n++)
				objects[node.subNode]->TraverseQuad(rays.Origin(n),rays.Dir(n),Output<otNormal,f32x4,i32x4>(output,n),node.id);
		}
	}
		
protected:
	void FindSplit(int nNode,const BVHBuilder&,vector<int> &indices);
	
	void Update();
	void UpdateBU(int node,int nParent);
	void UpdateTD(int node,int nParent);
	
	int AddNode(int sub,int count);
	
public:
	struct Node {
		template <class Real,class Vec>
		INLINE bool Test(Vec &rOrig,Vec &rDir,const Real &maxDist=Real(1.0f/0.0f)) const {
			return count ? bBox.Test(rOrig,rDir,maxDist) : optBBox.Test(rOrig,rDir,maxDist);
		}

		Matrix<Vec4f> trans,invTrans;
		Matrix<Vec4f> globalTrans;

		BBox bBox;
		OptBBox optBBox;
		
		int subNode,count,id;
	};
	
	vector<PObject> objects;
	vector<Node,AlignedAllocator<Node> > nodes;
};

#endif
