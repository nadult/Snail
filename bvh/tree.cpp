#include "bvh/tree.h"
#include <algorithm>


namespace {
	
	/*
	float BoxPointDistanceSq(const BBox &box,const Vec3f &point) const {
		Vec3f center=box.Center(),size=box.Size()*0.5f;
		float diff[3]={point.x-center.x,point.y-center.y,point.z-center.z};
		float length[3]={size.x,size.y,size.z};
		float p[3]={point.x,point.y,point.z};

		float distance = 0.0;
		float delta;

		for( int i=0; i<3; i++ )
			if ( diff[i]<-length[i] ) {
				delta=diff[i]+length[i];
				distance+=delta*delta;
				diff[i]=-length[i];
			}
			else if (diff[i]>length[i] ) {
				delta=diff[i]-length[i];
				distance+=delta*delta;
				diff[i]=length[i];
			}

		return distance;
	} */
};

static const BBox TriBox(const Triangle &tri) {
	return BBox(tri.BoundMin(), tri.BoundMax());
}

static float BoxSA(const BBox &box) {
	return (box.Width() * (box.Depth() + box.Height()) + box.Depth() * box.Height()) * 2.0f;
}

static int FindMaxAxis(Vec3f boxSize) {
	return boxSize.x > boxSize.y? boxSize.z > boxSize.x? 2 : 0 : boxSize.z > boxSize.y? 2 : 1;
}

void BVH::UpdateCache() {
	triCache.resize(elements.size());
	for(int n = 0; n < elements.size(); n++)
		triCache[n] = elements[n];
}

void BVH::FindSplit(int nNode, int first, int count, int depth) {
	if(depth <= 4) { printf("%d ", depth); fflush(stdout); }

	BBox bbox = nodes[nNode].bbox;
	enum { useSah = 1 };

	if(count <= 4) {
	LEAF_NODE:
		nodes[nNode].bbox = TriBox(elements[first]);
		for(int n = 1; n < count; n++)
			nodes[nNode].bbox += TriBox(elements[first + n]);
		depth = Max(depth, depth);
		nodes[nNode].first = first | 0x80000000;
		nodes[nNode].count = count;
	}
	else {
		int minIdx = count / 2, minAxis = FindMaxAxis(bbox.Size());
	
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
					rightSA[count - 1] = BoxSA(TriBox(elements[first + count - 1]));
					leftSA [0] = BoxSA(TriBox(elements[first]));

					BBox lastBox = TriBox(elements[first]);
					for(size_t n = 1; n < count; n++) {
						lastBox += TriBox(elements[first + n]);
						leftSA[n] = BoxSA(lastBox);
					}

					lastBox = TriBox(elements[first + count - 1]);
					for(int n = count - 2; n >= 0; n--) {
						lastBox += TriBox(elements[first + n]);
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
			
		BBox leftBox(TriBox(elements[first]));
		BBox rightBox(TriBox(elements[first + count - 1]));

		for(size_t n = 1; n < minIdx; n++)
			leftBox  += TriBox(elements[first + n]);
		for(size_t n = minIdx; n < count; n++)
			rightBox += TriBox(elements[first + n]);

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

		FindSplit(subNode + 0, first, minIdx, depth + 1);
		FindSplit(subNode + 1, first + minIdx, count - minIdx, depth + 1);
	}
}

BVH::BVH(const CompactTris &elems) {
	Construct(elems);
}

void BVH::Construct(const CompactTris &tElements) {
	elements = tElements;
	nodes.reserve(elements.size() * 2);
	nodes.clear();
	
	depth = 0;

	BBox bbox(elements[0].BoundMin(), elements[0].BoundMax());
	for(size_t n = 1; n < elements.size(); n++)
		bbox += BBox(elements[n].BoundMin(), elements[n].BoundMax());

	nodes.push_back(Node(bbox));
	FindSplit(0, 0, elements.size(), 0);
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
