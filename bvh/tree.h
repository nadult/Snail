#ifndef RTRACER_BVH_H
#define RTRACER_BVH_H

#include "ray_group.h"
#include "tree_stats.h"
#include "triangle.h"

class BVH {
public:
	BVH(const vector<Triangle, AlignedAllocator<Triangle> >&);
	BVH() { }
	void Construct(const TriVector&);
	void Serialize(Serializer&);
	const BBox GetBBox() const { return nodes[0].bbox; }

public:
	typedef Triangle   CElement;
	typedef ShTriangle SElement;

	enum { isComplex = 0 };
	enum { isctFlags = CElement::isctFlags | isct::fObject | isct::fStats };
	enum { filterSigns = 0 };

	const ShTriangle GetSElement(int elem, int subElem) const {
		const Triangle &tri = elements[elem];
		return ShTriangle(tri.P1(), tri.P2(), tri.P3(), {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f},
				tri.Nrm(), tri.Nrm(), tri.Nrm(), 0, 1);
	}

	template <int flags, int size, class Selector = int>
	void TraversePacket(Context<size,flags> &c, Selector s = Selector()) const {
		struct StackElem { int node, firstActive; };
		StackElem stack[maxDepth + 2]; int stackPos = 0;
		stack[stackPos++] = {0, 0};
		TreeStats<1> stats;

		Vec3f orig(c.Origin(0).x[0], c.Origin(0).y[0], c.Origin(0).z[0]);
		bool sign[3] = { c.Dir(0).x[0] < 0.0f, c.Dir(0).y[0] < 0.0f, c.Dir(0).z[0] < 0.0f };
		
		while(stackPos) {
			int nNode = stack[--stackPos].node;
			int firstActive = stack[stackPos].firstActive;

		CONTINUE:
			stats.LoopIteration();

			if(nodes[nNode].IsLeaf()) {
				int count = nodes[nNode].count, first = nodes[nNode].first;
				for(int n = 0; n < count; n++) {
					const Triangle &tri = elements[first + n];
					int ret = tri.Collide(c, first + n, firstActive);
					stats.Intersection(size);
				}
				continue;
			}
				
			bool test = 0; {
				const BBox &box = nodes[nNode].bbox;
				for(int q = 0; q < size; q++)
					if(box.TestI(c.Origin(q), c.IDir(q), c.Distance(q))) {
						firstActive = q;
						test = true;
						break;
					}
			}

			if(test) {
				int firstNode = nodes[nNode].firstNode ^ (sign[nodes[nNode].axis]?1 : 0) ^ gVals[0];
				int child = nodes[nNode].subNode;
				stack[stackPos++] = {child + (firstNode ^ 1), firstActive};
				nNode = child + firstNode;
				goto CONTINUE;
			}
		}

		if(c.stats)
			(*c.stats) += stats;
	}
		
protected:
	void FindSplit(int nNode, int first, int count, int depth);
	int maxDepth;
	
public:
	struct Node {
		Node() { }
		Node(const BBox &bbox) :bbox(bbox) { }
		bool IsLeaf() const { return subNode == 0; }

		BBox bbox;
		int subNode, first, count;
		short axis, firstNode;
	};

	vector<Triangle, AlignedAllocator<Triangle> > elements;
	vector<Node, AlignedAllocator<Node> > nodes;
};

SERIALIZE_AS_POD(BVH::Node)

#endif
