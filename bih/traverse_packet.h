
	template <class ElementContainer> template <int flags,int size> void
		Tree<ElementContainer>::TraversePacket0(Context<size,flags> &c) const
	{
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

		float sharedOrig[3];
		if(flags&isct::fShOrig) {
			sharedOrig[0]=c.Origin(0).x[0];
			sharedOrig[1]=c.Origin(0).y[0];
			sharedOrig[2]=c.Origin(0).z[0];
		}

		floatq tMin[size],tMax[size];
		for(int q=0;q<size;q++) {
			tMin[q]=ConstEpsilon<floatq>();
			tMax[q]=c.distance[q];
		}
		bBox.UpdateMinMaxDist(c.rays,dirMask,tMin,tMax);

		floatq fStackBegin[size*2*(maxLevel+2)],*fStack=fStackBegin;
		u32 nStackBegin[maxLevel+2],*nStack=nStackBegin;

		ObjectIdxBuffer<4> mailbox;

		const Node *node0=&nodes[0];
		int idx=0;

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
				if(fStack==fStackBegin) break;

				fStack-=size*2;
				for(int q=0;q<size;q++) tMin[q]=fStack[q];
				for(int q=0;q<size;q++) tMax[q]=Min(fStack[size+q],c.Distance(q));
				
				--nStack;
				idx=*nStack;
				continue;
			}

			const Node *node=node0+(idx&Node::idxMask);

			int axis=node->Axis();
			int nidx=dirMask&(1<<axis)?1:0;

			floatq near[size],far[size];
			f32x4b test1,test2; {
				const floatq *inv=(&c.rays.IDirPtr()->x)+axis;

				if(flags&isct::fShOrig) {
					float tnear=node->clip[0]-sharedOrig[axis],tfar=node->clip[1]-sharedOrig[axis];
					if(nidx) Swap(tnear,tfar);
					
					near[0]=Min( floatq(tnear)*inv[0], tMax[0]);
					far [0]=Max( floatq(tfar)*inv[0], tMin[0]);
					test1=tMin[0]>near[0];
					test2=tMax[0]<far[0];
					
					for(int q=1;q<size;q++) {
						near[q]=Min( floatq(tnear)*inv[q*3], tMax[q]);
						far [q]=Max( floatq(tfar)*inv[q*3], tMin[q]);
						
						test1=test1&&tMin[q]>near[q];
						test2=test2&&tMax[q]<far [q];
					}
				}
				else {
					const floatq *start=(&c.rays.OriginPtr()->x)+axis;
					float tnear=node->clip[0],tfar=node->clip[1];
					if(nidx) Swap(tnear,tfar);

					near[0]=Min( (floatq(tnear)-start[0])*inv[0], tMax[0]);
					far [0]=Max( (floatq(tfar) -start[0])*inv[0], tMin[0]);
					test1=tMin[0]>near[0];
					test2=tMax[0]<far[0];

					for(int q=1;q<size;q++) {
						near[q]=Min( (floatq(tnear)-start[q*3])*inv[q*3], tMax[q]);
						far [q]=Max( (floatq(tfar) -start[q*3])*inv[q*3], tMin[q]);
						
						test1=test1&&tMin[q]>near[q];
						test2=test2&&tMax[q]<far [q];
					}
				}
			}

			if(ForAll(test1)) {
				if(ForAll(test2)) goto POP_STACK;

				if(size==1) tMin[0]=Max(tMin[0],far[0]);
				else for(int q=0;q<size;q++) tMin[q]=far[q];
				idx=node->val[nidx^1];
				continue;
			}
			if(ForAll(test2)) {
				if(size==1) tMax[0]=Min(tMax[0],near[0]);
				else for(int q=0;q<size;q++) tMax[q]=near[q];
				idx=node->val[nidx];
				continue;
			}

			for(int q=0;q<size;q++) fStack[q]=far[q];
			for(int q=0;q<size;q++) {
				fStack[size+q]=tMax[q];
				tMax[q]=near[q];
			}
			fStack+=size*2;

			*nStack=node->val[nidx^1];
			nStack++;
		
			idx=node->val[nidx];
		}

		c.UpdateStats(stats);
	}

