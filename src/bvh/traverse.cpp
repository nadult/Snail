#include "bvh/tree.h"

namespace {

	struct StackElem {
		StackElem(int node, short first, short last) :node(node), firstActive(first), lastActive(last) { }
		StackElem() { }

		int node; short firstActive, lastActive;
	};

}

template <bool sharedOrigin, bool hasMask>
void BVH::TraversePrimaryN(Context<sharedOrigin, hasMask> &c) const {
	const int size = c.Size();
	StackElem stack[maxDepth + 2]; int stackPos = 0;
	stack[stackPos++] = StackElem(0, 0, (short)(size - 1));
	TreeStats stats;

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
			int count = nodes[nNode].count, first = nodes[nNode].first & 0x7fffffff;

			__builtin_prefetch(&tris[first + 0], 0, 3);
			__builtin_prefetch(&tris[first + 1], 0, 3);
			__builtin_prefetch(&tris[first + 2], 0, 3);
			__builtin_prefetch(&tris[first + 3], 0, 3);

			const BBox &box = nodes[nNode].bbox;
			if(!box.TestInterval(interval))
				continue;

			if(box.Test(c, firstActive, lastActive))
				for(int n = 0; n < count; n++) {
					const Triangle &tri = tris[first + n];
					if(sharedOrigin? /*tri.TestCornerRays(crays)*/ tri.TestInterval(interval) : 1) {
						tri.Collide(c, first + n, firstActive, lastActive);
						stats.Intersection(lastActive - firstActive + 1);
					}
				}

			continue;
		}
			
		int child = nodes[nNode].subNode;
		__builtin_prefetch(&nodes[child + 0], 0, 3);
		__builtin_prefetch(&nodes[child + 1], 0, 3);
			
		bool test = 0; {
			const BBox &box = nodes[nNode].bbox;
//			if(!box.TestCornerRays(crays))
			if(!box.TestInterval(interval))
				continue;
			test = box.Test(c, firstActive, lastActive);
		}

		if(test) {
			int firstNode = nodes[nNode].firstNode ^ sign[nodes[nNode].axis];
			stack[stackPos++] = StackElem(child + (firstNode ^ 1), firstActive, lastActive);
			nNode = child + firstNode;
			goto CONTINUE;
		}
	}

	if(c.stats)
		(*c.stats) += stats;
}

void BVH::TraverseShadow(ShadowContext &c) const {
	const int size = c.Size();
	TreeStats stats;

	StackElem stack[maxDepth + 2]; int stackPos = 0;
	stack[stackPos++] = StackElem(0, 0, (short)(size - 1));

	int sign[3] = { c.Dir(0).x[0] < 0.0f, c.Dir(0).y[0] < 0.0f, c.Dir(0).z[0] < 0.0f };
	RayInterval interval(c.rays, c.distance);
	CornerRays crays(c.rays);
	
	while(stackPos) {
		int nNode = stack[--stackPos].node;
		int firstActive = stack[stackPos].firstActive;
		int lastActive = stack[stackPos].lastActive;

	CONTINUE:
		stats.LoopIteration();

		if(nodes[nNode].IsLeaf()) {
			int count = nodes[nNode].count, first = nodes[nNode].first & 0x7fffffff;

			__builtin_prefetch(&tris[first + 0], 0, 3);
			__builtin_prefetch(&tris[first + 1], 0, 3);
			__builtin_prefetch(&tris[first + 2], 0, 3);
			__builtin_prefetch(&tris[first + 3], 0, 3);

			const BBox &box = nodes[nNode].bbox;
			if(!box.TestInterval(interval))
				continue;

			if(box.Test(c, firstActive, lastActive))
				for(int n = 0; n < count; n++) {
					const Triangle &tri = tris[first + n];
					if(tri.TestInterval(interval)/*tri.TestCornerRays(crays)*/) {
						if(tri.Collide(c, firstActive, lastActive)) {
							stats.Skip();
							if(c.stats) (*c.stats) += stats;
							return;
						}
						stats.Intersection(lastActive - firstActive + 1);
					}
				}
			continue;
		}
			
		int child = nodes[nNode].subNode;
		__builtin_prefetch(&nodes[child + 0], 0, 3);
		__builtin_prefetch(&nodes[child + 1], 0, 3);

		bool test = 0; {
			const BBox &box = nodes[nNode].bbox;
			if(!box.TestInterval(interval))
				continue;
			test = box.Test(c, firstActive, lastActive);
		}

		if(test) {
			int firstNode = nodes[nNode].firstNode ^ sign[nodes[nNode].axis];
			stack[stackPos++] = StackElem(child + (firstNode ^ 1), firstActive, lastActive);
			nNode = child + firstNode;
			goto CONTINUE;
		}
	}

	if(c.stats)
		(*c.stats) += stats;
}

template <bool sharedOrigin, bool hasMask>
void BVH::TraversePrimary(Context<sharedOrigin, hasMask> &c) const {
	const int size = c.Size();

	// Wyglada na to ze splitowanie jedynie spowalnia...
	TraversePrimaryN(c);
	return;


	if(sharedOrigin || size <= 4) {
		TraversePrimaryN(c);
		return;
	}

	bool split = 1;
	//TODO distant origins
	floatq dot = 1.0f;
	for(int q = 1; q < size; q++)
		dot = Condition(c.Distance(q) >= 0.0f, Min(dot, c.Dir(0) | c.Dir(q)), dot);
	split = ForAny(dot < (size == 64?0.99f : 0.95f));

	if(!split)
		TraversePrimaryN(c);

	if(split) {
		int size4 = size / 4;
		for(int k = 0; k < 4; k++) {
			Context<sharedOrigin, hasMask> split(RayGroup<sharedOrigin, hasMask>(c.rays, k * size4, size4),
					c.distance + k * size4, c.object + k * size4, c.element + k * size4, c.barycentric + k * size4);
			TraversePrimary(split);
		}
		if(c.stats) c.stats->Skip();
	}
}

template void BVH::TraversePrimaryN<0, 0>(Context<0, 0>&) const;
template void BVH::TraversePrimaryN<0, 1>(Context<0, 1>&) const;
template void BVH::TraversePrimaryN<1, 0>(Context<1, 0>&) const;
template void BVH::TraversePrimaryN<1, 1>(Context<1, 1>&) const;
	
template void BVH::TraversePrimary<0, 0>(Context<0, 0>&) const;
template void BVH::TraversePrimary<0, 1>(Context<0, 1>&) const;
template void BVH::TraversePrimary<1, 0>(Context<1, 0>&) const;
template void BVH::TraversePrimary<1, 1>(Context<1, 1>&) const;


void BVH::WideTrace(const Vec3f *origin, const Vec3f *dir, const Vec3f *idir,
						u16 *indices, float *dist, unsigned count, unsigned nodeIdx) const {
	const Node &node = nodes[nodeIdx];

	if(node.IsLeaf()) {
		int triCount = node.count, firstTri = node.first & 0x7fffffff;
		for(int t = 0; t < triCount; t++) {
			const Triangle &tri = tris[firstTri + t];
			for(unsigned n = 0; n < count; n++) {
				unsigned idx = indices[n];
				auto isct = tri.Collide<0>(origin[idx], dir[idx]);
				dist[idx] = Min(dist[idx], isct.Distance(0));
			}
		}
	}
	else {
		int child = node.subNode;
		const BBox &left  = nodes[child + 0].bbox;
		const BBox &right = nodes[child + 1].bbox;
		unsigned newCount;

		u16 newIndices[count];
		if(newCount = left.WideTest(origin, idir, indices, dist, count, newIndices))
			WideTrace(origin, dir, idir, newIndices, dist, newCount, child + 0);
		
		if(newCount = right.WideTest(origin, idir, indices, dist, count, newIndices))
			WideTrace(origin, dir, idir, newIndices, dist, newCount, child + 1);
	}
}


