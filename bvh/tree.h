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
	void PrintInfo() const;

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

		int sign[3] = { c.Dir(0).x[0] < 0.0f, c.Dir(0).y[0] < 0.0f, c.Dir(0).z[0] < 0.0f };
		Vec3f orig(c.Origin(0).x[0], c.Origin(0).y[0], c.Origin(0).z[0]);

		float minIDir[3], maxIDir[3]; Vec3f minDir, maxDir; {
			Vec3q imin = c.IDir(0), imax = c.IDir(0);
			Vec3q min = c.Dir(0), max = c.Dir(0);

			for(int q = 1; q < size; q++) {
				imin = VMin(imin, c.IDir(q));
				imax = VMax(imax, c.IDir(q));
				min = VMin(min, c.Dir(q));
				max = VMax(max, c.Dir(q));
			}
			for(int k = 0; k < 3; k++) {
				minIDir[k] = Minimize(imin[k]);
				maxIDir[k] = Maximize(imax[k]);
				minDir[k] = Minimize(min[k]);
				maxDir[k] = Maximize(max[k]);
			}
		}
		
		while(stackPos) {
			int nNode = stack[--stackPos].node;
			int firstActive = stack[stackPos].firstActive;

		CONTINUE:
			stats.LoopIteration();

			if(nodes[nNode].IsLeaf()) {
				const BBox &box = nodes[nNode].bbox;
				if(!box.TestInterval(orig, minIDir, maxIDir))
					continue;

				int count = nodes[nNode].count, first = nodes[nNode].first & 0x7fffffff;
				for(int n = 0; n < count; n++) {
					const Triangle &tri = elements[first + n];
					if(tri.TestInterval(orig, minDir, maxDir)) { //TODO: dokladniejszy test
						tri.Collide(c, first + n, firstActive);
						stats.Intersection(size);
					}
				}
				continue;
			}
				
			bool test = 0; {
				const BBox &box = nodes[nNode].bbox;
				if(!box.TestInterval(orig, minIDir, maxIDir))
					continue;

				for(int q = firstActive; q < size; q++)
					if(box.TestI(c.Origin(q), c.IDir(q), c.Distance(q))) {
						firstActive = q;
						test = true;
						break;
					}
			}

			if(test) {
				int firstNode = nodes[nNode].firstNode ^ sign[nodes[nNode].axis];
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
		bool IsLeaf() const { return subNode & 0x80000000; }

		BBox bbox;
		union { int subNode, first; };
		union { struct { short axis, firstNode; }; int count; };
	};

	vector<Triangle, AlignedAllocator<Triangle> > elements;
	vector<Node, AlignedAllocator<Node> > nodes;
};

static_assert(sizeof(BVH::Node) == 32, "Size of BVH::Node should be 32 bytes");
SERIALIZE_AS_POD(BVH::Node)

#endif
