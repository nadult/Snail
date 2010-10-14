#include "bvh/tree.h"
#include <algorithm>
#include "base_scene.h"

namespace {
	struct OrderTris {
		OrderTris(const ATriVector &tris, int axis)
			:tris(tris), axis(axis) { }

		bool operator()(int i1, int i2) const {
			// TODO: Jakis problem na GCC 4.3
			//na innych gcc usuniecie tego ifa powoduje wywalanie sie programu 
			if(i1 >= tris.size() || i2 >= tris.size()) {
				printf("!!! %d %d > %d\n", i1, i2, tris.size());
			}

			const Triangle &tri1 = tris[i1], &tri2 = tris[i2];

			Vec3f p1 = tri1.P1() + tri1.P2() + tri1.P3();
			Vec3f p2 = tri2.P1() + tri2.P2() + tri2.P3();
			return (&p1.x)[axis] < (&p2.x)[axis];
		}

		const ATriVector &tris;
		int axis;
	};

	struct TestTris {
		TestTris(const ATriVector &tris, int minAxis, int minIdx, float sub, float mul)
			:tris(tris), minAxis(minAxis), minIdx(minIdx), sub(sub), mul(mul) { }

		bool operator()(int idx) const {
			const Triangle &tri = tris[idx];

			Vec3f v1 = tri.P1(), v2 = tri.P2(), v3 = tri.P3();
			float f1 = (&v1.x)[minAxis];
			float f2 = (&v2.x)[minAxis];
			float f3 = (&v3.x)[minAxis];

			float center = (Min(f1, Min(f2, f3)) + Max(f1, Max(f2, f3))) * 0.5f;
			return int( (center - sub) * mul ) < minIdx;
		}

		const ATriVector &tris;
		int minAxis, minIdx;
		float sub, mul;
	};

	float BoxSA(const BBox &box) {
		return (box.Width() * (box.Depth() + box.Height()) + box.Depth() * box.Height()) * 2.0f;
	}

}

void BVH::FindSplitSweep(int nNode, int first, int count, int sdepth) {
	BBox bbox = nodes[nNode].bbox;
	enum { useSah = 0 };

	if(count <= 1) {
	LEAF_NODE:
		nodes[nNode].bbox = tris[first].GetBBox();
		for(int n = 1; n < count; n++)
			nodes[nNode].bbox += tris[first + n].GetBBox();
		depth = Max(depth, sdepth);
		nodes[nNode].first = first | 0x80000000;
		nodes[nNode].count = count;
	}
	else {
		int minIdx = count / 2, minAxis = MaxAxis(bbox.Size()); {
			vector<int> indices(count);
		
			if(useSah) {
				const float traverseCost = 0.0;
				const float intersectCost = 1.0;
				float minCost = constant::inf;
				float nodeSA = BoxSA(bbox);
				float noSplitCost = intersectCost * count * BoxSA(bbox);

				for(int axis = 0; axis <= 2; axis++) {
					for(int n = 0; n < count; n++)
						indices[n] = first + n;
					std::sort(&indices[0], &indices[count], OrderTris(tris, axis));

					vector<float> leftSA(count), rightSA(count); {
						rightSA[count - 1] = BoxSA(tris[indices[count - 1]].GetBBox());
						leftSA [0] = BoxSA(tris[indices[0]].GetBBox());

						BBox lastBox = tris[indices[0]].GetBBox();
						for(size_t n = 1; n < count; n++) {
							lastBox += tris[indices[n]].GetBBox();
							leftSA[n] = BoxSA(lastBox);
						}

						lastBox = tris[indices[count - 1]].GetBBox();
						for(int n = count - 2; n >= 0; n--) {
							lastBox += tris[indices[n]].GetBBox();
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

			if(!useSah || minAxis != 2) {
				for(int n = 0; n < count; n++)
					indices[n] = first + n;
				std::nth_element(indices.begin(), indices.begin() + minIdx, indices.end(),
									OrderTris(tris, minAxis));
			}

			vector<Triangle> ttemp(count);
			vector<ShTriangle> stemp(count);
			for(int n = 0; n < count; n++) {
				ttemp[n] = tris[indices[n]];
				stemp[n] = shTris[indices[n]];
			}
			for(int n = 0; n < count; n++) {
				tris[first + n] = ttemp[n];
				shTris[first + n] = stemp[n];
			}
		}

		BBox leftBox(tris[first].GetBBox());
		BBox rightBox(tris[first + count - 1].GetBBox());

		for(size_t n = 1; n < minIdx; n++)
			leftBox += tris[first + n].GetBBox();
		for(size_t n = minIdx; n < count; n++)
			rightBox += tris[first + n].GetBBox();

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

		FindSplitSweep(subNode + 0, first, minIdx, sdepth + 1);
		FindSplitSweep(subNode + 1, first + minIdx, count - minIdx, sdepth + 1);
	}
}

void BVH::FindSplit(int nNode, int first, int count, int sdepth) {
	BBox bbox = nodes[nNode].bbox;

	if(count <= 16) {
	LEAF_NODE:
		depth = Max(depth, sdepth);
		nodes[nNode].first = first | 0x80000000;
		nodes[nNode].count = count;
		return;
	}
	
	int subNode, leftCount, rightCount;
	{
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
			const BBox &box = tris[first + n].GetBBox();
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

		vector<int> indices(count);
		for(int n = 0; n < count; n++)
			indices[n] = first + n;
		
		std::partition(&indices[0], &indices[count], TestTris(tris, minAxis, minIdx, sub, mul));

		vector<Triangle> ttemp(count);
		vector<ShTriangle> stemp(count);
		for(int n = 0; n < count; n++) {
			ttemp[n] = tris[indices[n]];
			stemp[n] = shTris[indices[n]];
		}
		for(int n = 0; n < count; n++) {
			tris[first + n] = ttemp[n];
			shTris[first + n] = stemp[n];
		}

		BBox leftBox = leftBoxes[minIdx - 1];
		BBox rightBox = rightBoxes[minIdx];
		leftCount = leftCounts[minIdx - 1];
		rightCount = rightCounts[minIdx];

		if(leftCount == 0 || rightCount == 0) {
			minIdx = count / 2;
			leftBox = tris[first].GetBBox();
			rightBox = tris[first + count - 1].GetBBox();

			for(size_t n = 1; n < minIdx; n++)
				leftBox  += tris[first + n].GetBBox();
			for(size_t n = minIdx; n < count; n++)
				rightBox += tris[first + n].GetBBox();
			leftCount = minIdx;
			rightCount = count - leftCount;
		}
			
		subNode = nodes.size();
		nodes[nNode].subNode = subNode;

		nodes[nNode].axis = minAxis;
		nodes[nNode].firstNode = leftBox.min[minAxis] > rightBox.min[minAxis]? 1 : 0;
		nodes[nNode].firstNode =
			leftBox.min[minAxis] == rightBox.min[minAxis]? leftBox.max[minAxis] < rightBox.max[minAxis]? 0 :1 : 0;

		nodes.push_back(Node(leftBox));
		nodes.push_back(Node(rightBox));
	}

	FindSplit(subNode + 0, first, leftCount, sdepth + 1);
	FindSplit(subNode + 1, first + leftCount, rightCount, sdepth + 1);
}

BVH::BVH(const BaseScene &elems, bool fast) {
	Construct(elems, fast);
}

void BVH::Construct(const BaseScene &scene, bool fast) {
	tris = scene.ToTriVector();
	shTris = scene.ToShTriVector();
	InputAssert(shTris.size() == tris.size());

	nodes.reserve(tris.size() * 2);
	nodes.clear();
	
	depth = 0;

	BBox bbox(tris[0].GetBBox());
	for(size_t n = 1; n < tris.size(); n++)
		bbox +=	tris[n].GetBBox();

	nodes.push_back(Node(bbox));
	if(fast)
		FindSplit(0, 0, tris.size(), 0);
	else
		FindSplitSweep(0, 0, tris.size(), 0);
	InputAssert(depth <= maxDepth);

	std::map<int, string> matNames;
	const std::map<string, int> &tnames = scene.matNames;
	for(std::map<string, int>::const_iterator it = tnames.begin(); it != tnames.end(); ++it)
		matNames[it->second] = it->first;
	materials.resize(matNames.size());

	int idx = 0;
	for(std::map<int, string>::const_iterator it = matNames.begin(); it != matNames.end(); ++it) {
		InputAssert(it->first == idx);
		materials[idx++].name = it->second;
	}
}


void BVH::Serialize(Serializer &sr) {
	sr & depth & tris & shTris & nodes & materials;
}

void BVH::PrintInfo() const {
	double nodeBytes = nodes.size() * sizeof(Node);
	int objBytes = (sizeof(Triangle) + sizeof(ShTriangle)) * tris.size();

	printf("Triangles: %d (%.2fMB)\n", (int)tris.size(), objBytes * 0.000001);
	printf("Nodes: %8d * %2d = %6.2fMB\n", (int)nodes.size(), (int)sizeof(Node), nodeBytes * 0.000001);
	printf("~ %.0f bytes per triangle\n", double(nodeBytes + objBytes) / double(tris.size()));
	printf("Levels: %d\n\n", depth);
}

void BVH::UpdateMaterialIds(const std::map<string, int> &dict) {
	if(!materials.size()) {
		materials.push_back(MatId());
		return;
	}

	for(size_t n = 0; n < materials.size(); n++) {
		std::map<string, int>::const_iterator it = dict.find(materials[n].name);
		materials[n].id = it == dict.end() ? ~0 : it->second;
	}
}

