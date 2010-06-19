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

void BVH::FindSplit(int nNode, int first, int count, int depth) {
	BBox bbox = nodes[nNode].bbox;

	if(count <= 1) {
		maxDepth = Max(maxDepth, depth);
		nodes[nNode].subNode = 0;
		nodes[nNode].first = first;
		nodes[nNode].count = count;
	}
	else {
		int axis; {
			Vec3f size = bbox.Size();
			axis = size.x > size.y?0 : 1;
			if(size.z > size[axis]) axis = 2;
		}
		int mid = count / 2;
		if(mid == 0) mid = 1;

		std::nth_element(&elements[first], &elements[first + mid], &elements[first + count],
			[&](const Triangle &a, const Triangle &b) {
				float p1 = (&a.P1().x)[axis] + (&a.P2().x)[axis] + (&a.P3().x)[axis];
				float p2 = (&b.P1().x)[axis] + (&b.P2().x)[axis] + (&b.P3().x)[axis];
				return p1 < p2; } );
		
		BBox leftBox(elements[first].BoundMin(), elements[first].BoundMax());
		BBox rightBox(elements[first + count - 1].BoundMin(), elements[first + count - 1].BoundMax());

		for(size_t n = 1; n < mid; n++)
			leftBox  += BBox(elements[first + n].BoundMin(), elements[first + n].BoundMax());
		for(size_t n = mid; n < count; n++)
			rightBox += BBox(elements[first + n].BoundMin(), elements[first + n].BoundMax());

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
		nodes[nNode].axis = axis;
		nodes[nNode].firstNode = leftBox.min[axis] > rightBox.min[axis]? 1 : 0;
		nodes[nNode].firstNode =
			leftBox.min[axis] == rightBox.min[axis]? leftBox.max[axis] < rightBox.max[axis]? 0 :1 : 0;


		nodes.push_back(Node(leftBox));
		nodes.push_back(Node(rightBox));

		FindSplit(subNode + 0, first, mid, depth + 1);
		FindSplit(subNode + 1, first + mid, count - mid, depth + 1);
	}
}

BVH::BVH(const TriVector &elems) {
	Construct(elems);
}

void BVH::Construct(const vector<Triangle, AlignedAllocator<Triangle> > &tElements) {
	elements = tElements;
	nodes.reserve(elements.size() * 2);
	nodes.clear();
	
	maxDepth = 0;

	BBox bbox(elements[0].BoundMin(), elements[0].BoundMax());
	for(size_t n = 1; n < elements.size(); n++)
		bbox += BBox(elements[n].BoundMin(), elements[n].BoundMax());

	nodes.push_back(Node(bbox));
	FindSplit(0, 0, elements.size(), 0);
}

void BVH::Serialize(Serializer &sr) {
	sr & elements & nodes;
}
