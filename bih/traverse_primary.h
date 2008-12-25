

	template <class ElementContainer> template <int flags,int size> void
		Tree<ElementContainer>::TraversePrimary(Context<size,flags> &c) const
	{
		if(!(flags&isct::fShOrig)) ThrowException("BIH::TraversePrimary: origin must be shared");

		int dirMask=_mm_movemask_ps(_mm_shuffle_ps(_mm_shuffle_ps(c.Dir(0).x.m,c.Dir(0).y.m,0),
																	c.Dir(0).z.m,0+(2<<2)))&7;
	
		TreeStats<1> stats;	
		stats.TracingPacket(4*size);

		if(!CElement::isComplex&&flags&isct::fShadow&&c.shadowCache.Size()) {
			if(elements[c.shadowCache[0]].Collide(c,c.shadowCache[0])) {
				stats.Skip();
				c.UpdateStats(stats);
				return;
			}
			c.shadowCache.Clear();
		}

		float minInv[3],maxInv[3];
	   	ComputeMinMax<size>(c.rays.IDirPtr(),minInv,maxInv);

		float sharedOrig[3];
		sharedOrig[0]=c.Origin(0).x[0];
		sharedOrig[1]=c.Origin(0).y[0];
		sharedOrig[2]=c.Origin(0).z[0];
		float tMin=0.0f,tMax=1.0f/0.0f;

		bBox.UpdateMinMaxDist(sharedOrig,minInv,maxInv,dirMask,tMin,tMax);

		const Node *node0=&nodes[0];
		struct TPStackElem { float min,max; u32 idx; } stackBegin[maxLevel+1],*stack=stackBegin;
		int idx=0;

		ObjectIdxBuffer<4> mailbox;

		while(true) {
			stats.LoopIteration();

			if(idx&Node::leafMask) {
				idx&=Node::idxMask;

				if(!mailbox.Find(idx)) {
					mailbox.Insert(idx);

					stats.Intersection(size);
					int full=elements[idx].Collide(c,idx);
					if(!CElement::isComplex&&flags&isct::fShadow&&full) c.shadowCache.Insert(idx);
				}

			POP_STACK:
				if(stack==stackBegin) break;

				stack--;
				tMin=stack[0].min;
				f32x4 ttMin=c.Distance(0);
				for(int q=1;q<size;q++) ttMin=Max(ttMin,c.Distance(q));
				tMax=Min(Maximize(ttMin),stack[0].max);
				idx=stack[0].idx;
				continue;
			}

			const Node *node=node0+(idx&Node::idxMask);

			int axis=node->Axis();
			int nidx=dirMask&(1<<axis)?1:0;

			float near,far;
			{
				float tnear=node->clip[0]-sharedOrig[axis],tfar=node->clip[1]-sharedOrig[axis];
				float minI=minInv[axis],maxI=maxInv[axis];

				if(nidx) {
					Swap(tnear,tfar);
					near=Min(tnear*minI,tMax);
					far =Max(tfar *maxI,tMin); 
				}
				else {
					near=Min(tnear*maxI,tMax);
					far =Max(tfar *minI,tMin); 
				}
			}
			
			if(tMin>near) {
				if(tMax<far) goto POP_STACK;

				tMin=far;
				idx=node->val[nidx^1];
				continue;
			}
			if(tMax<far) {
				tMax=near;
				idx=node->val[nidx];
				continue;
			}

			stack[0].min=far;
			stack[0].max=tMax;
			tMax=near;

			stack[0].idx=node->val[nidx^1];
			stack++;
		
			idx=node->val[nidx];
		}

		c.UpdateStats(stats);
	}

