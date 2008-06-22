#ifndef RTRACER_KDTREE_H
#define RTRACER_KDTREE_H

#include "object.h"
#include "ray_group.h"
#include "tree_stats.h"
#include "context.h"


class SlowKDNode
{
public:
	SlowKDNode() :type(T_LEAF) { }
	SlowKDNode(int ax,float p,u32 c) :type((Type)ax),pos(p),child(c) { }

	enum Type { T_X=0, T_Y=1, T_Z=2, T_LEAF=3 } type;

	float pos;
	u32 child; // left: child+0    right: child+1
	vector<u32> objects;
};

typedef TTriangle<SlowEdgeNormals> KDTriangle;

class SlowKDTree
{
public:
	enum { MaxLevel=50 };
	typedef KDTriangle Object;

	SlowKDTree(const TriVector &objects);
	//void Draw(Image&,Vec3<float>,Vec3<float>,const Camera &cam,u32 node=0) const;

private:
	friend class KDTree;

	void Build(u32 node,u32 level,Vec3<float>,Vec3<float>);

	Vec3p pMin,pMax;
	vector<SlowKDNode> nodes;
	TriVector objects;
};

class KDNode
{
public:
	u32 ChildDist() const { return val&0x3fffffff; }
	u32 FirstObject() const { return val&0x3fffffff; }
	u32 NumObjects() const { return num; }

	u32 Axis() const { return (val>>30); }
	bool Leaf() const { return (val>>30)==3; }
	float Pos() const { return pos; }

	void SetLeaf(u32 first,u32 n) { num=n; val=first|(3u<<30); }
	void SetNode(u32 axis,float p,u32 nextAdv) { pos=p; val=nextAdv|(axis<<30); }


private:
	union { float pos; u32 num; };
	u32 val;
};

template <class Rays,class Selector> class TraverseContext;

class KDTree
{
public:
	enum { MaxLevel=SlowKDTree::MaxLevel };
	typedef KDTriangle Object;

	// Triangle flags usage:
	// Flag1:  1337: object is a blocker (there will be no split going through it) 0: normal
	// Flag2: object is full in node

	KDTree(const TriVector&) NOINLINE;
	KDTree(const SlowKDTree&);
	~KDTree();

	void Build(const SlowKDTree&);
	void PrintInfo() const;

	template <class Output,class Vec>
	void FullTraverse(const Vec &rOrigin,const Vec &rDir,const Output&) const NOINLINE;

	template <class Output>
	void Traverse(const Vec3p &origin,const Vec3p &dir,const Output&) const NOINLINE;

	template <class Output,class Rays,class Selector,class OtherContext>
	void TraverseFast(Rays&,const Selector&,const Output&,OtherContext*) const NOINLINE;

	template <class Output,class Rays,class Selector,class TContext>
	void TraverseSplitted(Rays&,const Selector&,const Output&,TContext*) const NOINLINE;

	template <class Output,class Rays,class Selector>
	void TraverseOptimized(Rays&,const Selector&,const Output&) const NOINLINE;

	template <class Output,class Rays,class Selector>
	void TraverseMono(Rays&,const Selector&,const Output&) const NOINLINE;

	bool Test() const;

	Vec3p pMin,pMax;
	TriVector objects;
	bool splittingFlag;

	vector<KDNode> nodes;
private:
	bool TestNode(Vec3f min,Vec3f max,int node) const;

	vector<u32> objectIds;
};

#include "kdtraverse_mono.h"
#include "kdtraversal.h"


#endif

