#pragma once

#include "ray_group.h"
#include "tree_stats.h"
#include "triangle.h"
#include <map>

class BaseScene;


class BVH {
public:
	BVH(const BaseScene&, int flags = fastBuild | useSah);
	BVH() { }
	void Construct(const BaseScene&, int flags = fastBuild | useSah);
	Ex<void> save(FileStream&) const;
	Ex<void> load(FileStream&);
	const BBox GetBBox() const { return nodes[0].bbox; }
	void PrintInfo() const;

	enum {
		noShadingData = 1,
		fastBuild = 2,
		useSah = 4
	};

public:
	typedef Triangle   CElement;
	typedef ShTriangle SElement;

	enum { isComplex = 0 };
	enum { isctFlags = CElement::isctFlags | isct::fObject | isct::fStats };
	enum { maxDepth = 64 };

	bool HasShadingData() const { return shTris.size(); }

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
		string name;
		int id;
	};

	void UpdateMaterialIds(const std::map<string, int>&);

	int GetMaterialId(int idx, int) const {
		return materials[idx].id;
	}

	ATriVector tris;
	AShTriVector shTris;
	using ANodesVector = std::vector<Node, AlignedAllocator<Node, 256> >;
	ANodesVector nodes;

	vector<MatId> materials;
	int depth;
};

SERIALIZE_AS_POD(BVH::Node)
