

	template <class ElementContainer> template <int flags>
	void Tree<ElementContainer>::TraverseMono(FContext<flags> &c) const
	{
		TreeStats<1> stats;
		stats.TracingRay();

		int dirMask=(c.iDir.x<0.0f?1:0)+(c.iDir.y<0.0f?2:0)+(c.iDir.z<0.0f?4:0);
		float tMin=0.0f,tMax=c.Distance(0);
		bBox.UpdateMinMaxDist(&c.origin.x,&c.iDir.x,&c.iDir.x,dirMask,tMin,tMax);

		struct Locals { float tMin,tMax; u32 idx; } stackBegin[maxLevel+2],*stack=stackBegin;
		const Node *node,*node0=&nodes[0];
		int idx=0;


		while(true) {
			stats.LoopIteration();
			if(tMin>tMax) goto POP_STACK;
		
			if(idx&Node::leafMask) {
				idx&=Node::idxMask;
				stats.Intersection();
				elements[idx].Collide(c,idx);
POP_STACK:
				if(stack==stackBegin) break;
				stack--;
				tMin=stack->tMin;
				tMax=Min(stack->tMax,c.Distance(0));
				idx=stack->idx;
				continue;
			}

			idx&=Node::idxMask;
			node=node0+idx;
			int axis=node->Axis();
			int nidx=dirMask&(1<<axis)?1:0,fidx=nidx^1;

			float near,far; {
				float start=(&c.origin.x)[axis],inv=(&c.iDir.x)[axis];
				near=(node->clip[nidx]-start)*inv;
				far =(node->clip[fidx]-start)*inv;
			}

			if(tMin>near) {
				if(tMax<far) goto POP_STACK;

				tMin=Max(tMin,far);
				idx=node->val[fidx];
				continue;
			}
			if(tMax<far) {
				tMax=Min(tMax,near);
				idx=node->val[nidx];
				continue;
			}

			stack->tMin=Max(tMin,far);
			stack->tMax=tMax;
			stack->idx=node->val[fidx];
			stack++;

			tMax=Min(tMax,near);
			idx=node->val[nidx];
		}

		c.UpdateStats(stats);
	}

