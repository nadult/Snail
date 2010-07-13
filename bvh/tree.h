#ifndef RTRACER_BVH_H
#define RTRACER_BVH_H

#include "ray_group.h"
#include "tree_stats.h"
#include "triangle.h"

class BVH {
public:
	BVH(const CompactTris&);
	BVH() { }
	void Construct(const CompactTris&);
	void Serialize(Serializer&);
	const BBox GetBBox() const { return nodes[0].bbox; }
	void PrintInfo() const;
	void UpdateCache();

public:
	typedef Triangle   CElement;
	typedef ShTriangle SElement;

	enum { isComplex = 0 };
	enum { isctFlags = CElement::isctFlags | isct::fObject | isct::fStats };
	enum { maxDepth = 32 };

	const ShTriangle GetSElement(int elem, int subElem) const {
		return elements.GetSElement(elem, subElem);
	}

	int EPSearch(const RayInterval&, int start = 0) const;
	bool FindAny(const RayInterval&, int start) const;

	template <bool sharedOrigin, bool hasMask>
	void TraversePrimary0(Context<sharedOrigin, hasMask> &c, int firstNode = 0) const;

	template <bool sharedOrigin, bool hasMask>
	void TraverseShadow0(Context<sharedOrigin, hasMask> &c, int firstNode = 0) const;
	
	template <bool sharedOrigin, bool hasMask>
	void TraversePrimary(Context<sharedOrigin, hasMask> &c) const;
	
//	template <int flags,template <int> class Selector>
//	void TraverseShadow(Context<4,flags> &c,const Selector<4> &selector) const {
//		TraverseShadow0(c, selector);
//	}

	template <bool sharedOrigin, bool hasMask>
	void TraverseShadow(Context<sharedOrigin, hasMask> &c) const;

protected:
	void FindSplit(int nNode, int first, int count, int depth);
	int depth;
	
public:
	struct Node {
		Node() { }
		Node(const BBox &bbox) :bbox(bbox) { }
		bool IsLeaf() const { return subNode & 0x80000000; }

		BBox bbox;
		union { int subNode, first; };
		union { struct { short axis, firstNode; }; int count; };
	};

	CompactTris elements;
	vector<Triangle> triCache;
	vector<Node, AlignedAllocator<Node> > nodes;
};

SERIALIZE_AS_POD(BVH::Node)

#endif
