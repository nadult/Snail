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

	template <bool sharedOrigin, bool hasMask>
	void TraversePrimary0(Context<sharedOrigin, hasMask> &c) const {
		const int size = c.Size();

		struct StackElem { int node; short firstActive, lastActive; };
		StackElem stack[maxDepth + 2]; int stackPos = 0;
		stack[stackPos++] = {0, 0, (short)(size - 1)};
		TreeStats<1> stats;

		int sign[3] = { c.Dir(0).x[0] < 0.0f, c.Dir(0).y[0] < 0.0f, c.Dir(0).z[0] < 0.0f };

		CornerRays crays(c.rays);
		RayInterval interval(c.rays);
		
		while(stackPos) {
			int nNode = stack[--stackPos].node;
			int firstActive = stack[stackPos].firstActive;
			int lastActive = stack[stackPos].lastActive;

		CONTINUE:
			stats.LoopIteration();

			if(nodes[nNode].IsLeaf()) {
				const BBox &box = nodes[nNode].bbox;

				int count = nodes[nNode].count, first = nodes[nNode].first & 0x7fffffff;
				for(int n = 0; n < count; n++) {
					const Triangle &tri = elements[first + n];
					if(sharedOrigin && !hasMask? tri.TestCornerRays(crays) : gVals[0] || tri.TestInterval(interval)) {
						tri.Collide(c, first + n, firstActive, lastActive);
						stats.Intersection(size);
					}
				}
				continue;
			}
				
			bool test = 0; {
				const BBox &box = nodes[nNode].bbox;
				if(!box.TestInterval(interval))
					continue;

				for(int q = firstActive; q < size; q++)
					if(c.Mask(q) && box.TestI(c.Origin(q), c.IDir(q), c.Distance(q), c.SSEMask(q))) {
						firstActive = q;
						test = 1;
						break;
					}
				for(int q = lastActive; q > firstActive; q--)
					if(c.Mask(q) && box.TestI(c.Origin(q), c.IDir(q), c.Distance(q), c.SSEMask(q))) {
						lastActive = q;
						test = 1;
						break;
					}
			}

			if(test) {
				int firstNode = nodes[nNode].firstNode ^ sign[nodes[nNode].axis];
				int child = nodes[nNode].subNode;
				stack[stackPos++] = {child + (firstNode ^ 1), (short)firstActive, (short)lastActive};
				nNode = child + firstNode;
				goto CONTINUE;
			}
		}

		if(c.stats)
			(*c.stats) += stats;
	}

	template <bool sharedOrigin, bool hasMask>
	void TraverseShadow0(Context<sharedOrigin, hasMask> &c, int firstNode = 0) const {
		const int size = c.Size();

		struct StackElem { int node; short firstActive, lastActive; };
		StackElem stack[maxDepth + 2]; int stackPos = 0;
		stack[stackPos++] = {firstNode, 0, (short)(size - 1) };
		TreeStats<1> stats;

		int sign[3] = { c.Dir(0).x[0] < 0.0f, c.Dir(0).y[0] < 0.0f, c.Dir(0).z[0] < 0.0f };
		if(c.shadowCache[0] != ~0) {
			const Triangle &tri = elements[c.shadowCache[0]];
			tri.CollideShadow(c, 0, size - 1);
		}

		RayInterval interval(c.rays);
		
		while(stackPos) {
			int nNode = stack[--stackPos].node;
			int firstActive = stack[stackPos].firstActive;
			int lastActive = stack[stackPos].lastActive;

		CONTINUE:
			stats.LoopIteration();

			if(nodes[nNode].IsLeaf()) {
				const BBox &box = nodes[nNode].bbox;

				int count = nodes[nNode].count, first = nodes[nNode].first & 0x7fffffff;
				for(int n = 0; n < count; n++) {
					const Triangle &tri = elements[first + n];
					if(tri.TestInterval(interval)) {
						tri.CollideShadow(c, firstActive, lastActive);
						c.shadowCache.Insert(first + n);
						stats.Intersection(size);
					}
				}
				continue;
			}
				
			bool test = 0; {
				const BBox &box = nodes[nNode].bbox;
				if(!box.TestInterval(interval))
					continue;

				for(int q = firstActive; q < size; q++)
					if(c.Mask(q) && box.TestI(c.Origin(q), c.IDir(q), c.Distance(q), c.SSEMask(q))) {
						firstActive = q;
						test = 1;
						break;
					}
				for(int q = lastActive; q > firstActive; q--)
					if(c.Mask(q) && box.TestI(c.Origin(q), c.IDir(q), c.Distance(q), c.SSEMask(q))) {
						lastActive = q;
						test = 1;
						break;
				}
			}

			if(test) {
				int firstNode = nodes[nNode].firstNode ^ sign[nodes[nNode].axis];
				int child = nodes[nNode].subNode;
				stack[stackPos++] = {child + (firstNode ^ 1), (short)firstActive, (short)lastActive};
				nNode = child + firstNode;
				goto CONTINUE;
			}
		}

		if(c.stats)
			(*c.stats) += stats;
	}
	
	template <bool sharedOrigin, bool hasMask>
	void TraversePrimary(Context<sharedOrigin, hasMask> &c) const {
		const int size = c.Size();
		bool split = 1;

		RaySelector selector(c.MaskPtr(), c.Size());
		bool selectorsFiltered = 1;
		if(hasMask) {
			const int size4 = size / 4; //TODO: ...
			bool any = 0;
			for(int n = 0; n < size4; n++) {
				int mask = selector.Mask4(n);
				selectorsFiltered &= mask == 0x0f0f0f0f;
				any |= mask;
			}
			if(!any)
				return;
		}

		if(!hasMask || selectorsFiltered) {
			//TODO distant origins
			floatq dot = 1.0f;
			for(int q = 1; q < size; q++)
				dot = Min(dot, c.Dir(0) | c.Dir(q));
			split = ForAny(dot < 0.99);

			if(!split)
				TraversePrimary0(c);
		}

		if(split) {
		//	if(gVals[0]) {
				RaySelector temp = selector;
				int start = 0;
				while(true) {
					char buf[c.Size() + 4];
					RaySelector newSel(buf, c.Size());
					newSel.Clear();

					for(;start < size; start++)
						if(temp[start]) break;
					if(start == size)
						break;
	
					Vec3q lead;
					for(int i = 0; i < 4; i++)
						if(temp[start] & (1 << i))
							lead = (Vec3q)ExtractN(c.Dir(start), i);
					for(int q = start; q < size; q++) {
						int mask = ForWhich((c.Dir(q) | lead) >= 0.9) & temp[q];
						temp[q] &= ~mask;
						newSel[q] = mask;
					}
					//TODO: nowy selektor
				//	TraversePrimary0(Context, newSel);
				}
		//	}
		//	else {
				//TODO: split
			//	for(int q = 0; q < 4; q++) {
			//		Context<size/4,flags> subC(c.Split(q));
			//		TraversePrimary0(subC, selector.SubSelector(q));
			//	}
		//	}
			if(c.stats) c.stats->Skip();
		}
	}
	
//	template <int flags,template <int> class Selector>
//	void TraverseShadow(Context<4,flags> &c,const Selector<4> &selector) const {
//		TraverseShadow0(c, selector);
//	}

	template <bool sharedOrigin, bool hasMask>
	void TraverseShadow(Context<sharedOrigin, hasMask> &c) const {
		TraverseShadow0(c);

		/*
		bool split = 1;

		bool selectorsFiltered = size <= 64;
		if(!Selector<size>::full) {
			bool any = 0;
			for(int n = 0; n < size/4; n++) {
				int mask = selector.Mask4(n);
				selectorsFiltered &= mask == 0x0f0f0f0f;
				any |= mask;
			}
			if(!any)
				return;
		}

		if((Selector<size>::full || selectorsFiltered) && size <= 64) {
			floatq dot = 1.0f;
			for(int q = 1; q < size; q++)
				dot = Min(dot, c.Dir(0) | c.Dir(q));
			split = ForAny(dot < 0.95f);

			if(!split)
				TraverseShadow0(c, selector);
		}

		if(split) {
			for(int q = 0; q < 4; q++) {
				Context<size/4,flags> subC(c.Split(q));
				subC.shadowCache = c.shadowCache;
				TraverseShadow0(subC, selector.SubSelector(q));
				c.shadowCache = subC.shadowCache;
			}
			if(c.stats) c.stats->Skip();
		}*/
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
