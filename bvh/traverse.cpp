#include "bvh/tree.h"

template <bool sharedOrigin, bool hasMask>
void BVH::TraversePrimary0(Context<sharedOrigin, hasMask> &c, int firstNode) const {
	const int size = c.Size();

	struct StackElem { int node; short firstActive, lastActive; };
	StackElem stack[maxDepth + 2]; int stackPos = 0;
	stack[stackPos++] = { firstNode, 0, (short)(size - 1)};
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
					if(tri.TestCornerRays(crays)) {
						tri.Collide(c, first + n, firstActive, lastActive);
						stats.Intersection(lastActive - firstActive + 1);
					}
				}

			continue;
		}
			
		int child = nodes[nNode].subNode;
		_mm_prefetch(&nodes[child], _MM_HINT_T0);
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
void BVH::TraverseShadow0(Context<sharedOrigin, hasMask> &c, int firstNode) const {
	const int size = c.Size();

	struct StackElem { int node; short firstActive, lastActive; };
	StackElem stack[maxDepth + 2]; int stackPos = 0;
	stack[stackPos++] = {firstNode, 0, (short)(size - 1) };
	TreeStats<1> stats;

	int sign[3] = { c.Dir(0).x[0] < 0.0f, c.Dir(0).y[0] < 0.0f, c.Dir(0).z[0] < 0.0f };
//	if(c.shadowCache[0] != ~0) {
//		const Triangle &tri = triCache[c.shadowCache[0]];
//		tri.CollideShadow(c, 0, size - 1);
//	}

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
					if(tri.TestInterval(interval)) {
						tri.CollideShadow(c, firstActive, lastActive);
				//		c.shadowCache.Insert(first + n);
						stats.Intersection(size);
					}
				}
				continue;
		}
			
		int child = nodes[nNode].subNode;
		_mm_prefetch(&nodes[child], _MM_HINT_T0);
			
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
			char tempData[size + 4];
			RaySelector temp(tempData, size);
			for(int n = 0; n < temp.Size(); n++)
				temp[n] = c.Mask(n);
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
					if(temp[start] & (1 << i)) {
						lead = (Vec3q)ExtractN(c.Dir(start), i);
						break;
					}
				for(int q = start; q < size; q++) {
					int mask = ForWhich((c.Dir(q) | lead) >= 0.9) & temp[q];
					temp[q] &= ~mask;
					newSel[q] = mask;
				}
				Context<sharedOrigin, 1> splitCtx(RayGroup<sharedOrigin, 1>(c.rays.OriginPtr(),
								c.rays.DirPtr(), c.rays.IDirPtr(), size, buf), c.distance, c.object, c.element);
				TraversePrimary0(splitCtx);
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
void BVH::TraverseShadow(Context<sharedOrigin, hasMask> &c) const {
	TraverseShadow0(c);
	return;

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

	//	if(!split)
			TraverseShadow0(c);
	}

	if(split) {
	//	if(gVals[0]) {
			char tempData[size + 4];
			RaySelector temp(tempData, size);
			for(int n = 0; n < temp.Size(); n++)
				temp[n] = c.Mask(n);
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
					if(temp[start] & (1 << i)) {
						lead = (Vec3q)ExtractN(c.Dir(start), i);
						break;
					}
				for(int q = start; q < size; q++) {
					int mask = ForWhich((c.Dir(q) | lead) >= 0.9) & temp[q];
					temp[q] &= ~mask;
					newSel[q] = mask;
				}
				Context<sharedOrigin, 1> splitCtx(RayGroup<sharedOrigin, 1>(c.rays.OriginPtr(),
								c.rays.DirPtr(), c.rays.IDirPtr(), size, buf), c.distance, c.object, c.element);
				TraverseShadow0(splitCtx);
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
//	if(gVals[1]) {
//		TraverseShadow0(c);
/*		return;
	}
	const int size = c.Size();
	bool split = 1;
	bool selectorsFiltered = 1;
	if(hasMask) {
		bool any = 0;
		for(int n = 0; n < size; n++) {
			int mask = c.Mask(n);
			selectorsFiltered &= mask == 0x0f;
			any |= mask;
		}
		if(!any)
			return;
	}

	if(hasMask || selectorsFiltered) {
		floatq dot = 1.0f;
		for(int q = 1; q < size; q++)
			dot = Min(dot, c.Dir(0) | c.Dir(q));
		split = ForAny(dot < 0.99f);

		if(!split)
			TraverseShadow0(c);
	}

	if(split) {
		const int size4 = c.Size() / 4;
		for(int q = 0; q < 4; q++) {
			Context<sharedOrigin, hasMask> subC(RayGroup<sharedOrigin, hasMask>(c.rays, size4 * q, size4),
					c.distance, c.object, c.element, c.stats);
			subC.shadowCache = c.shadowCache;
			TraverseShadow0(subC);
			c.shadowCache = subC.shadowCache;
		}
		if(c.stats) c.stats->Skip();
	} */
}

template void BVH::TraversePrimary0<0, 0>(Context<0, 0>&, int) const;
template void BVH::TraversePrimary0<0, 1>(Context<0, 1>&, int) const;
template void BVH::TraversePrimary0<1, 0>(Context<1, 0>&, int) const;
template void BVH::TraversePrimary0<1, 1>(Context<1, 1>&, int) const;

template void BVH::TraverseShadow0<0, 0>(Context<0, 0>&, int) const;
template void BVH::TraverseShadow0<0, 1>(Context<0, 1>&, int) const;
template void BVH::TraverseShadow0<1, 0>(Context<1, 0>&, int) const;
template void BVH::TraverseShadow0<1, 1>(Context<1, 1>&, int) const;
	
template void BVH::TraversePrimary<0, 0>(Context<0, 0>&) const;
template void BVH::TraversePrimary<0, 1>(Context<0, 1>&) const;
template void BVH::TraversePrimary<1, 0>(Context<1, 0>&) const;
template void BVH::TraversePrimary<1, 1>(Context<1, 1>&) const;
	
template void BVH::TraverseShadow<0, 0>(Context<0, 0>&) const;
template void BVH::TraverseShadow<0, 1>(Context<0, 1>&) const;
template void BVH::TraverseShadow<1, 0>(Context<1, 0>&) const;
template void BVH::TraverseShadow<1, 1>(Context<1, 1>&) const;

