#ifndef RTRACER_BVH_H
#define RTRACER_BVH_H

#include "ray_group.h"
#include "tree_stats.h"
#include "triangle.h"

class BVH {
public:
	BVH(const vector<Triangle, AlignedAllocator<Triangle> >&);
	const BBox GetBBox() const { return nodes[0].bbox; }

public:

	template <int flags, int size>
	void TraversePacket(Context<size,flags> &c, int nNode = 0) const {
		const Node &node=nodes[nNode];
		//output.stats->Skip();
		
		enum { shared = 1 };
		int stack[maxDepth + 2], stackPos;
		stack[stackPos++] = nNode;
		TreeStats<1> stats;
		
		while(stackPos) {
			nNode = stack[--stackPos];
			const Node &node = nodes[nNode];
			stats.LoopIteration();

			if(node.IsLeaf()) {
				for(int n = 0; n < node.count; n++) {
					const Triangle &tri = elements[node.first + n];

					auto isct = tri.Collide(c.rays);
					for(int q = 0; q < size; q++) {
						auto mask = isct.Distance(q) < c.Distance(q);
						if(ForAny(mask)) {
							c.Distance(q) = Condition(mask, isct.Distance(q), c.Distance(q));
							c.Element(q) = Condition(mask, isct.Element(q), c.Element(q));
						}
					}
				}
				continue;
			}
				
			bool test = 0;
			for(int q = 0; q < size && !test; q++)
				test |= node.bbox.TestI(c.Origin(shared?0 : q), c.IDir(q), c.Distance(q));

			if(test) {
				stack[stackPos++] = node.subNode + 0;
				stack[stackPos++] = node.subNode + 1;
			}
		}

		if(c.stats)
			c.stats->Update(stats);
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
	};

	vector<Triangle, AlignedAllocator<Triangle> > elements;
	vector<Node, AlignedAllocator<Node> > nodes;
};


#endif
