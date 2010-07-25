#ifndef RTRACER_BVH_H
#define RTRACER_BVH_H

#include "ray_group.h"
#include "tree_stats.h"
#include "triangle.h"

class BVH {
public:
	BVH(const CompactTris&, const std::map<string, int> &matNames, bool fast = 1);
	BVH() { }
	void Construct(const CompactTris&, const std::map<string, int> &matNames, bool fast = 1);
	void Serialize(Serializer&);
	const BBox GetBBox() const { return nodes[0].bbox; }
	void PrintInfo() const;
	void UpdateCache();

public:
	typedef Triangle   CElement;
	typedef ShTriangle SElement;

	enum { isComplex = 0 };
	enum { isctFlags = CElement::isctFlags | isct::fObject | isct::fStats };
	enum { maxDepth = 64 };

	const ShTriangle GetSElement(int elem, int subElem) const {
		return elements.GetSElement(elem, subElem);
	}

	template <bool sharedOrigin, bool hasMask>
	void TraversePrimaryN(Context<sharedOrigin, hasMask> &c) const;
	
	template <bool sharedOrigin, bool hasMask>
	void TraversePrimary(Context<sharedOrigin, hasMask> &c) const;
	
	void TraverseShadow(ShadowContext &c) const;
	

protected:
	void FindSplit(int nNode, int first, int count, int depth);
	void FindSplitSweep(int nNode, int first, int count, int depth);
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

	struct MatId {
		MatId() :id(~0) { }
		void Serialize(Serializer &sr) { sr & name; }

		string name;
		int id;
	};

	void UpdateMaterialIds(const std::map<string, int>&);

	int GetMaterialId(int idx, int) const {
		return materials[idx].id;
	}

	vector<Triangle> triCache;
	vector<MatId> materials;

	CompactTris elements;
	vector<Node, AlignedAllocator<Node> > nodes;
};

SERIALIZE_AS_POD(BVH::Node)

#endif
