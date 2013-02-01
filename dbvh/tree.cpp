#include "dbvh/tree.h"
#include <algorithm>

void ObjectInstance::ComputeBBox() {
	BBox b = tree->GetBBox();
	floatq x0(b.min.x);
	floatq x1(b.max.x);
	floatq y(b.min.y, b.min.y, b.max.y, b.max.y);
	floatq z(b.min.z, b.max.z, b.min.z, b.max.z);
	Vec3q p0(	y * rotation[0].y + z * rotation[0].z,
				y * rotation[1].y + z * rotation[1].z,
				y * rotation[2].y + z * rotation[2].z);
	Vec3q p1(	x1 * rotation[0].x + p0.x,
				x1 * rotation[1].x + p0.y,
				x1 * rotation[2].x + p0.z);
	p0.x += x0 * rotation[0].x;
	p0.y += x0 * rotation[1].x;
	p0.z += x0 * rotation[2].x;
	bbox = BBox(Minimize(VMin(p0, p1)) + translation, Maximize(VMax(p0, p1)) + translation);
}
namespace {

	struct TestBoxes {
		TestBoxes(int minAxis, int minIdx, float sub, float mul)
			:minAxis(minAxis), minIdx(minIdx), sub(sub), mul(mul) { }

		bool operator()(const ObjectInstance &box) const {
			return int( (box.Center(minAxis) - sub) * mul ) < minIdx;
		}

		int minAxis, minIdx;
		float sub, mul;
	};

	float BoxSA(const BBox &box) {
		return (box.Width() * (box.Depth() + box.Height()) + box.Depth() * box.Height()) * 2.0f;
	}

}

void DBVH::FindSplit(int nNode, int first, int count, int sdepth) {
	BBox bbox = nodes[nNode].bbox;

	if(count <= 1) {
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

		int nBins = count < 8? 8 : 16;
		Bin bins[nBins];

		float mul = nBins * (1.0f - constant::epsilon) /
			((&bbox.max.x)[minAxis] - (&bbox.min.x)[minAxis]);
		float sub = (&bbox.min.x)[minAxis];

		for(int n = 0; n < count; n++) {
			const BBox &box = elements[first + n].GetBBox();
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
		
		ObjectInstance *it =
			std::partition(&elements[first], &elements[first + count], TestBoxes(minAxis, minIdx, sub, mul));

		BBox leftBox = leftBoxes[minIdx - 1];
		BBox rightBox = rightBoxes[minIdx];
		int leftCount = leftCounts[minIdx - 1];
		int rightCount = rightCounts[minIdx];

		if(leftCount == 0 || rightCount == 0) {
			minIdx = count / 2;
			leftBox = elements[first].GetBBox();
			rightBox = elements[first + count - 1].GetBBox();

			for(size_t n = 1; n < minIdx; n++)
				leftBox  += elements[first + n].GetBBox();
			for(size_t n = minIdx; n < count; n++)
				rightBox += elements[first + n].GetBBox();
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

DBVH::DBVH(const vector<ObjectInstance> &elems) {
	Construct(elems);
}

void DBVH::Construct(const vector<ObjectInstance> &tElements) {
	elements = tElements;
	nodes.reserve(elements.size() * 2);
	nodes.clear();
	
	depth = 0;

	BBox bbox(elements[0].GetBBox());
	for(size_t n = 1; n < elements.size(); n++)
		bbox += BBox(elements[n].GetBBox());

	nodes.push_back(Node(bbox));
	FindSplit(0, 0, elements.size(), 0);

	ASSERT(depth <= maxDepth);
}

void DBVH::PrintInfo() const {
	double nodeBytes = nodes.size() * sizeof(Node);
	double objBytes = elements.size() * sizeof(ObjectInstance);

	printf("Instances: %d * %d (%.2fMB)\n",
			(int)elements.size(), sizeof(ObjectInstance), objBytes * 0.000001);
	printf("Nodes: %d * %d = %6.2fMB\n",
			(int)nodes.size(), (int)sizeof(Node), nodeBytes * 0.000001);
	printf("~ %.0f bytes per instance\n", double(nodeBytes + objBytes) / double(elements.size()));
	printf("Levels: %d\n\n", depth);
}

