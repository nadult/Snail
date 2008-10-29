#ifndef RTRACER_OBJECT_H
#define RTRACER_OBJECT_H

#include "rtbase.h"
#include "sphere.h"
#include "triangle.h"
#include "context.h"
#include "ray_group.h"

class BIHTree;

class Object {
public:
	virtual ~Object() { }
	virtual void TraverseMono(const Vec3p &rOrigin,const Vec3p &rDir,Output<otNormal,float,u32> output,int) const=0;
	virtual void TraverseQuad(const Vec3q &rOrigin,const Vec3q &rDir,Output<otNormal,f32x4,i32x4> output,int) const=0;
	virtual BBox GetBBox() const=0;

	template <class Derived>
	Vec3f TFlatNormals(u32 elementId)  {
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
	
	virtual Vec3f FlatNormals(u32) { return Vec3f(0,1,0); }
};

extern vector<Object*> gObjects;


class BVH {
public:
	BBox GetBBox() const;
	void TraverseMono(const Vec3p &rOrigin,const Vec3p &rDir,Output<otNormal,float,u32> output,int node=0) const;
	void TraverseQuad(const Vec3q &rOrigin,const Vec3q &rDir,Output<otNormal,f32x4,i32x4> output,int node=0) const;

	template <class Rays>
	void TraverseMonoGroup(Rays &group,const RaySelector<Rays::size> &sel,const Output<otNormal,f32x4,i32x4> &out) const {
		Vec3p orig[4],dir[4];
		u32 tmp[4];

		if(Rays::sharedOrigin)
			Convert(group.Origin(sel[0]),orig);
		
		for(int i=0;i<sel.Num();i++) {
			int q=sel[i];
			Vec3p dir[4];
			int bmask=sel.BitMask(i);

			if(!Rays::sharedOrigin)
				Convert(group.Origin(q),orig);
			Convert(group.Dir(q),dir);

			float *dist=(float*)(out.dist+q);
			u32 *objId =Output<otNormal,f32x4,i32x4>::objectIndexes?(u32*)(out.object+q):tmp;
			u32 *elemId=Output<otNormal,f32x4,i32x4>::objectIndexes?(u32*)(out.element+q):tmp;

			if(bmask&1) TraverseMono(orig[0],dir[0],::Output<otNormal,float,u32>(dist+0,objId+0,elemId+0,out.stats));
			if(bmask&2) TraverseMono(orig[1],dir[1],::Output<otNormal,float,u32>(dist+1,objId+1,elemId+1,out.stats));
			if(bmask&4) TraverseMono(orig[2],dir[2],::Output<otNormal,float,u32>(dist+2,objId+2,elemId+2,out.stats));
			if(bmask&8) TraverseMono(orig[3],dir[3],::Output<otNormal,float,u32>(dist+3,objId+3,elemId+3,out.stats));
		}
	}
	
	template <class Rays>
	void TraverseQuadGroup(Rays &rays,const RaySelector<Rays::size> &sel,const Output<otNormal,f32x4,i32x4> &out) const {
		for(int i=0;i<sel.Num();i++) {
			int q=sel[i];
			TraverseQuad(rays.Origin(q),rays.Dir(q),Output<otNormal,f32x4,i32x4>(out.dist+q,out.object+q,out.element+q,out.stats));
		}
	}
			
	void UpdateBox(int node=0);
	void UpdateGlobalTrans(int node=0,Matrix<Vec4f> *parentMat=0);
	
	int AddNode(const Matrix<Vec4f> &m,int sub,int count);
	
	struct Node {
		Matrix<Vec4f> trans,invTrans;
		Matrix<Vec4f> globalTrans;

		BBox box;
	
		Vec3f obbAxis[3],obbCenter;
		float obbExtent[3],sRadius;
		
		int subNode,count,id;
	};
	
	vector<Node,AlignedAllocator<Node> > nodes;
};

extern BVH *gBVH;

#endif
