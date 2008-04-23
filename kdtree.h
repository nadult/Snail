#ifndef KDTREE_H
#define KDTREE_H

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

class SlowKDTree
{
public:
	enum { MaxLevel=40 };
	typedef Triangle Object;

	SlowKDTree(const vector<Object> &objects);
	//void Draw(Image&,Vec3<float>,Vec3<float>,const Camera &cam,u32 node=0) const;

private:
	friend class KDTree;

	void Build(u32 node,u32 level,Vec3<float>,Vec3<float>);

	Vec3p pMin,pMax;
	vector<SlowKDNode> nodes;
	vector<Object> objects;
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

class KDTree
{
public:
	enum { MaxLevel=SlowKDTree::MaxLevel };
	typedef Triangle Object;

	KDTree(const vector<Object>&);
	KDTree(const SlowKDTree&);
	~KDTree();

	void Build(const SlowKDTree&);
	void PrintInfo() const;

	template <class Output,class Vec,class base>
	void FullTraverse(const Vec &rOrigin,const Vec &rDir,const Output&) const;


	template <class Output>
	void TraverseMono(const Vec3p &rOrigin,const Vec3p &rDir,const Output &out) const NOINLINE;

	template <class Output,class Group>
	void TraverseFast(Group &group,const RaySelector<Group::size>&,const floatq &maxD,const Output &out) const NOINLINE;


	template <class Output,class Group,class Selector>
	void TraverseOptimized(Group &group,const Selector&,const floatq &maxD,const Output &out,bool primary=1) const NOINLINE;

	template <class Output,class Group>
	void TraverseMonoGroup(Group &group,const RaySelector<Group::size>&,const Output &out) const;


	template <class Group>
	int GetDepth(Group &group,const RaySelector<Group::size>&) const;

	bool TestNode(Vec3f min,Vec3f max,int node) const;
	bool Test() const;

	Vec3p pMin,pMax;
	vector<Object> objects;

//private:
	vector<KDNode> nodes;
	vector<u32> objectIds;
};

#include "kdtraversal.h"


#endif

