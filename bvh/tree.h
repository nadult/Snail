#ifndef RTRACER_BVH_H
#define RTRACER_BVH_H

#include "ray_group.h"
#include "tree_stats.h"
#include "triangle.h"

class BaseScene;
	

class BVH {
public:
	BVH(const BaseScene&, bool fast = 1, bool useSah = 1);
	BVH() { }
	void Construct(const BaseScene&, bool fast = 1, bool useSah = 1);
	void Serialize(Serializer&);
	const BBox GetBBox() const { return nodes[0].bbox; }
	void PrintInfo() const;

public:
	typedef Triangle   CElement;
	typedef ShTriangle SElement;

	enum { isComplex = 0 };
	enum { isctFlags = CElement::isctFlags | isct::fObject | isct::fStats };
	enum { maxDepth = 64 };

	const ShTriangle &GetSElement(int elem, int subElem) const {
		return shTris[elem];
	}
	const Vec3f GetNormal(int elem, int subElem) const {
		return tris[elem].Nrm();
	}

	template <bool sharedOrigin, bool hasMask>
	void TraversePrimaryN(Context<sharedOrigin, hasMask> &c) const;
	
	template <bool sharedOrigin, bool hasMask>
	void TraversePrimary(Context<sharedOrigin, hasMask> &c) const;
	
	void TraverseShadow(ShadowContext &c) const;
	
	void WideTrace(const Vec3f *origin, const Vec3f *dir, const Vec3f *idir,
					u16 *indices, float *dist, unsigned count, unsigned nodeIdx = 0) const;

protected:
	void FindSplit(int nNode, int first, int count, int depth);
	void FindSplitSweep(int nNode, int first, int count, int depth, bool useSah);
	
public:
	struct Node {
		Node() { }
		Node(const BBox &bbox) :bbox(bbox) { }
		bool IsLeaf() const { return subNode & 0x80000000; }

		BBox bbox;
		union { int subNode, first; };
#ifdef __BIG_ENDIAN
		union { struct { short firstNode, axis; }; int count; };
#else
		union { struct { short axis, firstNode; }; int count; };
#endif
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

	ATriVector tris;
	AShTriVector shTris;
	vector<Node, AlignedAllocator<Node, 256> > nodes;

	vector<MatId> materials;
	int depth;
};

SERIALIZE_AS_POD(BVH::Node)

#endif
