#ifndef RTRACER_BIHTREE_H
#define RTRACER_BIHTREE_H

#include "object.h"
#include "ray_group.h"
#include "tree_stats.h"
#include "context.h"

typedef TTriangle<SlowEdgeNormals> BIHTriangle;

// Liscie nie sa przechowywane w drzewie
// Zamiast odnosnika do liscia jest od razu odnosnik do obiektu
class BIHNode {
public:
	enum { leafMask=1<<29, idxMask=0x1fffffff };

	inline i32 Child1() const { return val[0]&0x1fffffff; }
	inline i32 Child2() const { return val[1]&0x1fffffff; }
	inline i32 Child(uint idx) const { return val[idx]&0x1fffffff; }

	inline bool Child1IsLeaf() const { return val[0]&leafMask; }
	inline bool Child2IsLeaf() const { return val[1]&leafMask; }
	inline bool ChildIsLeaf(uint idx) const { return val[idx]&leafMask; }

	inline int Axis() const { return (val[0]>>30); }
	inline float ClipLeft() const { return clip[0]; }
	inline float ClipRight() const { return clip[1]; }

	inline bool HasChild1() const { return val[1]&(1<<30); }
	inline bool HasChild2() const { return val[1]&(1<<31); }

	float clip[2];
	u32 val[2];
};

class BIHIdx {
public:
	BIHIdx() { }
	BIHIdx(int i,const Vec3f &mi,const Vec3f &ma,float mul) :idx(i),min(mi),max(ma) {
		Vec3f vsize=max-min;
		size=Max(vsize.x,Max(vsize.y,vsize.z))*mul;
	}

	Vec3f min,max;
	float size;
	i32 idx;
};


template <class TObject=BIHTriangle>
class BIHTree {
public:
	typedef TObject Object;
	enum { maxLevel=60 };

	BIHTree(const vector<Object> &obj);

	void PrintInfo() const;
	uint FindSimilarParent(vector<u32> &parents,uint nNode,uint axis) const;
	void Build(vector<BIHIdx> &indices,vector<u32> &parents,uint nNode,int first,int last,Vec3p min,Vec3p max,uint level);

	template <class Output>
	void TraverseMono(const Vec3p &rOrigin,const Vec3p &tDir,Output output) const;

	template <class Output>
	void TraverseQuad(const Vec3q &rOrigin,const Vec3q &tDir,Output output,int dirMask) const;
	template <class Output,bool shared>
	void TraverseQuad4(const Vec3q *rOrigin,const Vec3q *tDir,floatq *out,i32x4 *object,TreeStats *tstats,int dirMask) const;
	template <class Output>
	void TraverseQuad4Primary(const Vec3q *rOrigin,const Vec3q *tDir,floatq *out,i32x4 *object,TreeStats *tstats,int dirMask) const;

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

	template <class Output,class Rays>
	void TraverseQuadGroup(Rays &rays,const RaySelector<Rays::size> &sel,const Output &out,int dirMask) const {
		int i=0;
		for(;i+3<sel.Num();i+=4) {
			int q[4]={sel[i],sel[i+1],sel[i+2],sel[i+3]};
			Vec3q tOrig[4],tDir[4];
			floatq dist[4]; i32x4 obj[4];

			dist[0]=out.dist[q[0]]; dist[1]=out.dist[q[1]];
			dist[2]=out.dist[q[2]]; dist[3]=out.dist[q[3]];
			if(Output::objectIndexes) {
				obj[0]=out.object[q[0]]; obj[1]=out.object[q[1]];
				obj[2]=out.object[q[2]]; obj[3]=out.object[q[3]];
			}
			tOrig[0]=rays.Origin(q[0]); tOrig[1]=rays.Origin(q[1]);
			tOrig[2]=rays.Origin(q[2]); tOrig[3]=rays.Origin(q[3]);
			tDir[0]=rays.Dir(q[0]); tDir[1]=rays.Dir(q[1]);
			tDir[2]=rays.Dir(q[2]); tDir[3]=rays.Dir(q[3]);

			if(Output::type==otPrimary)
				 TraverseQuad4Primary<Output>(tOrig,tDir,dist,obj,out.stats,dirMask);
			else
				TraverseQuad4<Output,Rays::sharedOrigin>(tOrig,tDir,dist,obj,out.stats,dirMask);

			out.dist[q[0]]=dist[0]; out.dist[q[1]]=dist[1];
			out.dist[q[2]]=dist[2]; out.dist[q[3]]=dist[3];
			if(Output::objectIndexes) {
				out.object[q[0]]=obj[0]; out.object[q[1]]=obj[1];
				out.object[q[2]]=obj[2]; out.object[q[3]]=obj[3];
			}
		}
		for(;i<sel.Num();i++) {
			int q=sel[i];
			TraverseQuad(rays.Origin(q),rays.Dir(q),Output(out.dist+q,out.object+q,out.stats),dirMask);
		}
	}


	template <class Output,class Rays>
	void TraverseOptimized(Rays &rays,const RaySelector<Rays::size> &sel,const Output &out) const {
		if(!sel.Num()) return;

		RaySelector<Rays::size> selectors[9];
		rays.GenSelectors(sel,selectors);

		for(int k=0;k<8;k++) {
			RaySelector<Rays::size> &sel=selectors[k];
			if(sel.Num()) {
				// Ulepszyc, ta czworka moze byc popsuta (wektorki w roznych kierunkach)
				/*Vec3q dir0=rays.Dir(sel[0]);

				if(Output::type!=otPrimary) for(int i=1;i<sel.Num();i++) {
					int q=sel[i];

					int bitMask=sel.BitMask(i);
					if(CountMaskBits(bitMask)<4) { selectors[8].Add(q,bitMask); sel.Disable(i--); continue; }
	
					floatq dot=rays.Dir(q)|dir0;
					if(ForAny(dot<Const<floatq,500,1000>())) { selectors[8].Add(q); sel.Disable(i--); continue; }
				}*/
				TraverseQuadGroup(rays,sel,out,k);
			}
		}
		if(selectors[8].Num())
			TraverseMonoGroup(rays,selectors[8],out);
	}

	Vec3p pMin,pMax;
	vector<BIHNode> nodes;
	Vector<BIHTriangle> objects;
};

#include "bihtrav_mono.h"
#include "bihtrav_quad.h"
#include "bihtrav_quad4.h"
#include "bihtrav_quad4p.h"
#include "bihtree.inl"

#endif

