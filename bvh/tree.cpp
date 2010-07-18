#include "bvh/tree.h"
#include <algorithm>


static float BoxSA(const BBox &box) {
	return (box.Width() * (box.Depth() + box.Height()) + box.Depth() * box.Height()) * 2.0f;
}

void BVH::UpdateCache() {
	triCache.resize(elements.size());
	for(int n = 0; n < elements.size(); n++)
		triCache[n] = elements[n];
}

void BVH::FindSplitSweep(int nNode, int first, int count, int depth) {
	BBox bbox = nodes[nNode].bbox;
	enum { useSah = 1 };

	if(count <= 1) {
	LEAF_NODE:
		nodes[nNode].bbox = elements[first].GetBBox();
		for(int n = 1; n < count; n++)
			nodes[nNode].bbox += elements[first + n].GetBBox();
		depth = Max(depth, depth);
		nodes[nNode].first = first | 0x80000000;
		nodes[nNode].count = count;
	}
	else {
		int minIdx = count / 2, minAxis = MaxAxis(bbox.Size());
	
		if(useSah) {
			const float traverseCost = 0.0;
			const float intersectCost = 1.0;
			float minCost = constant::inf;
			float nodeSA = BoxSA(bbox);
			float noSplitCost = intersectCost * count * BoxSA(bbox);

			for(int axis = 0; axis <= 2; axis++) {
				std::sort(&elements.tris[first], &elements.tris[first + count],
				[&elements, axis](const CompactTris::TriIdx &a, const CompactTris::TriIdx &b) {
					float p1 = (&elements.verts[a.v1].x)[axis] + (&elements.verts[a.v2].x)[axis]
										+ (&elements.verts[a.v3].x)[axis];
					float p2 = (&elements.verts[b.v1].x)[axis] + (&elements.verts[b.v2].x)[axis]
										+ (&elements.verts[b.v3].x)[axis];
					return p1 < p2; } );

				vector<float> leftSA(count), rightSA(count); {
					rightSA[count - 1] = BoxSA(elements[first + count - 1].GetBBox());
					leftSA [0] = BoxSA(elements[first].GetBBox());

					BBox lastBox = elements[first].GetBBox();
					for(size_t n = 1; n < count; n++) {
						lastBox += elements[first + n].GetBBox();
						leftSA[n] = BoxSA(lastBox);
					}

					lastBox = elements[first + count - 1].GetBBox();
					for(int n = count - 2; n >= 0; n--) {
						lastBox += elements[first + n].GetBBox();
						rightSA[n] = BoxSA(lastBox);
					}
				}

				for(size_t n = 1; n < count; n++) {
					float cost = leftSA[n - 1] * n + rightSA[n] * (count - n);
					if(cost < minCost) {
						minCost = cost;
						minIdx = n;
						minAxis = axis;
					}
				}
			}

			minCost = traverseCost + intersectCost * minCost;
			if(noSplitCost < minCost)
				goto LEAF_NODE;
		}

		if(!useSah || minAxis != 2)
			std::nth_element(&elements.tris[first], &elements.tris[first + minIdx], &elements.tris[first + count],
			[&elements, minAxis](const CompactTris::TriIdx &a, const CompactTris::TriIdx &b) {
				float p1 = (&elements.verts[a.v1].x)[minAxis] + (&elements.verts[a.v2].x)[minAxis]
									+ (&elements.verts[a.v3].x)[minAxis];
				float p2 = (&elements.verts[b.v1].x)[minAxis] + (&elements.verts[b.v2].x)[minAxis]
									+ (&elements.verts[b.v3].x)[minAxis];
				return p1 < p2; } );
			
		BBox leftBox(elements[first].GetBBox());
		BBox rightBox(elements[first + count - 1].GetBBox());

		for(size_t n = 1; n < minIdx; n++)
			leftBox  += elements[first + n].GetBBox();
		for(size_t n = minIdx; n < count; n++)
			rightBox += elements[first + n].GetBBox();

		int subNode = nodes.size();
		nodes[nNode].subNode = subNode;

	//	float maxDiff = -constant::inf;
	//	for(int ax = 0; ax < 3; ax++) {
	//		float diff = Max(leftBox.min[axis] - rightBox.max[axis], rightBox.min[axis] - leftBox.max[axis]);
	//		if(diff > maxDiff) {
	//			maxDiff = diff;
	//			axis = ax;
	//		}
	//	}
		nodes[nNode].axis = minAxis;
		nodes[nNode].firstNode = leftBox.min[minAxis] > rightBox.min[minAxis]? 1 : 0;
		nodes[nNode].firstNode =
			leftBox.min[minAxis] == rightBox.min[minAxis]? leftBox.max[minAxis] < rightBox.max[minAxis]? 0 :1 : 0;

		nodes.push_back(Node(leftBox));
		nodes.push_back(Node(rightBox));

		FindSplitSweep(subNode + 0, first, minIdx, depth + 1);
		FindSplitSweep(subNode + 1, first + minIdx, count - minIdx, depth + 1);
	}
}

void BVH::FindSplit(int nNode, int first, int count, int sdepth) {
	BBox bbox = nodes[nNode].bbox;

	if(count <= 4) {
	LEAF_NODE:
		depth = Max(depth, sdepth);
		nodes[nNode].first = first | 0x80000000;
		nodes[nNode].count = count;
	}
	else {
		int minAxis = MaxAxis(bbox.Size());
		const float traverseCost = 0.0;
		const float intersectCost = 1.0;

		struct Bin {
			Bin() :count(0) {
				box.min = Vec3f(constant::inf, constant::inf, constant::inf);
				box.max = Vec3f(-constant::inf, -constant::inf, -constant::inf);
			}

			BBox box;
			int count;
		};

		enum { nBins = 16 };
		Bin bins[nBins];

		float mul = nBins * (1.0f - constant::epsilon) /
			((&bbox.max.x)[minAxis] - (&bbox.min.x)[minAxis]);
		float sub = (&bbox.min.x)[minAxis];

		for(int n = 0; n < count; n++) {
			const BBox &box = elements.GetBBox(first + n);
			float c = ((&box.max.x)[minAxis] + (&box.min.x)[minAxis]) * 0.5f;
			int nBin = int( (c - sub) * mul );
			bins[nBin].count++;
			bins[nBin].box += box;
		}

		BBox leftBoxes[nBins], rightBoxes[nBins];
		int leftCounts[nBins], rightCounts[nBins];

		rightBoxes[nBins - 1] = bins[nBins - 1].box;
		rightCounts[nBins - 1] = bins[nBins - 1].count;
		leftBoxes[0] = bins[0].box;
		leftCounts[0] = bins[0].count;

		for(size_t n = 1; n < nBins; n++) {
			leftBoxes[n] = leftBoxes[n - 1] + bins[n].box;
			leftCounts[n] = leftCounts[n - 1] + bins[n].count;
		}
		for(int n = nBins - 2; n >= 0; n--) {
			rightBoxes[n] = rightBoxes[n + 1] + bins[n].box;
			rightCounts[n] = rightCounts[n + 1] + bins[n].count;
		}
		
		float minCost = constant::inf;
		float noSplitCost = intersectCost * count * BoxSA(bbox);
		int minIdx = 1;

		for(size_t n = 1; n < nBins; n++) {
			float cost =
				(leftCounts[n - 1]?	BoxSA(leftBoxes[n - 1]) * leftCounts[n - 1] : 0) +
				(rightCounts[n]?	BoxSA(rightBoxes[n]) * rightCounts[n] : 0);

			if(cost < minCost) {
				minCost = cost;
				minIdx = n;
			}
		}

		minCost = traverseCost + intersectCost * minCost;
		if(noSplitCost < minCost)
			goto LEAF_NODE;
		
		auto it = std::partition(&elements.tris[first], &elements.tris[first + count],
				[&elements, minAxis, minIdx, sub, mul](const CompactTris::TriIdx &tri) {
					float v1 = (&elements.verts[tri.v1].x)[minAxis];
					float v2 = (&elements.verts[tri.v2].x)[minAxis];
					float v3 = (&elements.verts[tri.v3].x)[minAxis];
					float center = (Min(v1, Min(v2, v3)) + Max(v1, Max(v2, v3))) * 0.5f;
					return int( (center - sub) * mul ) < minIdx;
				});

		BBox leftBox = leftBoxes[minIdx - 1];
		BBox rightBox = rightBoxes[minIdx];
		int leftCount = leftCounts[minIdx - 1];
		int rightCount = rightCounts[minIdx];

		if(leftCount == 0 || rightCount == 0) {
			minIdx = count / 2;
			leftBox = elements.GetBBox(first);
			rightBox = elements.GetBBox(first + count - 1);

			for(size_t n = 1; n < minIdx; n++)
				leftBox  += elements.GetBBox(first + n);
			for(size_t n = minIdx; n < count; n++)
				rightBox += elements.GetBBox(first + n);
			leftCount = minIdx;
			rightCount = count - leftCount;
		}
			
		int subNode = nodes.size();
		nodes[nNode].subNode = subNode;

		nodes[nNode].axis = minAxis;
		nodes[nNode].firstNode = leftBox.min[minAxis] > rightBox.min[minAxis]? 1 : 0;
		nodes[nNode].firstNode =
			leftBox.min[minAxis] == rightBox.min[minAxis]? leftBox.max[minAxis] < rightBox.max[minAxis]? 0 :1 : 0;

		nodes.push_back(Node(leftBox));
		nodes.push_back(Node(rightBox));

		FindSplit(subNode + 0, first, leftCount, sdepth + 1);
		FindSplit(subNode + 1, first + leftCount, rightCount, sdepth + 1);
	}
}

BVH::BVH(const CompactTris &elems, bool fast) {
	Construct(elems, fast);
}

void BVH::Construct(const CompactTris &tElements, bool fast) {
	elements = tElements;
	nodes.reserve(elements.size() * 2);
	nodes.clear();
	
	depth = 0;

	BBox bbox(elements.GetBBox(0));
	for(size_t n = 1; n < elements.size(); n++)
		bbox +=	elements.GetBBox(n);

	nodes.push_back(Node(bbox));
	if(fast)
		FindSplit(0, 0, elements.size(), 0);
	else
		FindSplitSweep(0, 0, elements.size(), 0);
	InputAssert(depth <= maxDepth);
}

void BVH::Serialize(Serializer &sr) {
	sr & depth & elements & nodes;
}

void BVH::PrintInfo() const {
	double nodeBytes = nodes.size() * sizeof(Node);
	double objBytes = elements.MemSize();

	printf("Triangles: %d (%.2fMB)\n", (int)elements.size(), objBytes * 0.000001);
	printf("Nodes: %8d * %2d = %6.2fMB\n", (int)nodes.size(), (int)sizeof(Node), nodeBytes * 0.000001);
	printf("~ %.0f bytes per triangle\n", double(nodeBytes + objBytes) / double(elements.size()));
	printf("Levels: %d\n\n", depth);
}
