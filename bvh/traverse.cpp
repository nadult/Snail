#include "bvh/tree.h"

template <bool sharedOrigin, bool hasMask>
void BVH::TraversePrimaryN(Context<sharedOrigin, hasMask> &c) const {
	const int size = c.Size();

	struct StackElem { int node; short firstActive, lastActive; };
	StackElem stack[maxDepth + 2]; int stackPos = 0;
	stack[stackPos++] = { 0, 0, (short)(size - 1)};
	TreeStats stats;

	int sign[3] = { c.Dir(0).x[0] < 0.0f, c.Dir(0).y[0] < 0.0f, c.Dir(0).z[0] < 0.0f };

//	CornerRays crays(c.rays);
	RayInterval interval(c.rays);
	
	while(stackPos) {
		int nNode = stack[--stackPos].node;
		int firstActive = stack[stackPos].firstActive;
		int lastActive = stack[stackPos].lastActive;

	CONTINUE:
		stats.LoopIteration();

		if(nodes[nNode].IsLeaf()) {
			int count = nodes[nNode].count, first = nodes[nNode].first & 0x7fffffff;

			_mm_prefetch(&triCache[first + 0], _MM_HINT_T0);
			_mm_prefetch(&triCache[first + 1], _MM_HINT_T0);
			_mm_prefetch(&triCache[first + 2], _MM_HINT_T0);
			_mm_prefetch(&triCache[first + 3], _MM_HINT_T0);

			const BBox &box = nodes[nNode].bbox;
			if(!box.TestInterval(interval))
				continue;

			if(box.Test(c, firstActive, lastActive))
				for(int n = 0; n < count; n++) {
					const Triangle &tri = triCache[first + n];
					if(sharedOrigin?tri.TestInterval(interval):1/*tri.TestCornerRays(crays)*/) {
						tri.Collide(c, first + n, firstActive, lastActive);
						stats.Intersection(lastActive - firstActive + 1);
					}
				}

			continue;
		}
			
		int child = nodes[nNode].subNode;
		_mm_prefetch(&nodes[child + 0], _MM_HINT_T0);
		_mm_prefetch(&nodes[child + 1], _MM_HINT_T0);
			
		bool test = 0; {
			const BBox &box = nodes[nNode].bbox;
			if(!box.TestInterval(interval))
				continue;
			test = box.Test(c, firstActive, lastActive);
		}

		if(test) {
			int firstNode = nodes[nNode].firstNode ^ sign[nodes[nNode].axis];
			stack[stackPos++] = {child + (firstNode ^ 1), (short)firstActive, (short)lastActive};
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

	struct StackElem { int node; short firstActive, lastActive; };
	StackElem stack[maxDepth + 2]; int stackPos = 0;
	stack[stackPos++] = { 0, 0, (short)(size - 1) };

	int sign[3] = { c.Dir(0).x[0] < 0.0f, c.Dir(0).y[0] < 0.0f, c.Dir(0).z[0] < 0.0f };
	RayInterval interval(c.rays, c.distance);
	
	while(stackPos) {
		int nNode = stack[--stackPos].node;
		int firstActive = stack[stackPos].firstActive;
		int lastActive = stack[stackPos].lastActive;

	CONTINUE:
		stats.LoopIteration();

		if(nodes[nNode].IsLeaf()) {
			int count = nodes[nNode].count, first = nodes[nNode].first & 0x7fffffff;

			_mm_prefetch(&triCache[first + 0], _MM_HINT_T0);
			_mm_prefetch(&triCache[first + 1], _MM_HINT_T0);
			_mm_prefetch(&triCache[first + 2], _MM_HINT_T0);
			_mm_prefetch(&triCache[first + 3], _MM_HINT_T0);

			const BBox &box = nodes[nNode].bbox;
			if(!box.TestInterval(interval))
				continue;

			if(box.Test(c, firstActive, lastActive))
				for(int n = 0; n < count; n++) {
					const Triangle &tri = triCache[first + n];
					if(1 || tri.TestInterval(interval)) {
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
		_mm_prefetch(&nodes[child + 0], _MM_HINT_T0);
		_mm_prefetch(&nodes[child + 1], _MM_HINT_T0);

		bool test = 0; {
			const BBox &box = nodes[nNode].bbox;
			if(!box.TestInterval(interval))
				continue;
			test = box.Test(c, firstActive, lastActive);
		}

		if(test) {
			int firstNode = nodes[nNode].firstNode ^ sign[nodes[nNode].axis];
			stack[stackPos++] = {child + (firstNode ^ 1), (short)firstActive, (short)lastActive};
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

