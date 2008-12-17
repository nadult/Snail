
	template <class ElementContainer> template <template<int,int> class Rays,int flags,int size>
		Isct<f32x4,size,Tree<ElementContainer>::isctFlags|flags>
		Tree<ElementContainer>::TraversePacket0(const Rays<size,flags> &rays) const
	{
		static_assert(flags&isct::fInvDir,"");

		Isct<f32x4,size,isctFlags|flags> out;

		for(int q=0;q<size;q++)
			out.Distance(q)=rays.MaxDist(q);

		int dirMask=_mm_movemask_ps(_mm_shuffle_ps(_mm_shuffle_ps(rays.Dir(0).x.m,rays.Dir(0).y.m,0),
																	rays.Dir(0).z.m,0+(2<<2)))&7;
		TreeStats<1> &stats=out.Stats();
		stats.TracingPacket(4*size);

		if(flags&isct::fShadow&&rays.lastShadowTri!=-1&&rays.lastShadowTri<elements.size()) {
		//	stats.Skip();
			f32x4b mask[size];
			
			//TODO: if all rays hit, lastShadowTri=idx
			Isct<f32x4,size,CElement::isctFlags|flags>
				tOut=elements[rays.lastShadowTri].Collide(rays);

			for(int q=0;q<size;q++) {
				mask[q]=tOut.Distance(q)<out.Distance(q);
				out.Distance(q)=Condition(mask[q],floatq(0.0001f),out.Distance(q));
			}

			f32x4b test=mask[0];
			for(int q=1;q<size;q++) test=test&&mask[q];
			
			if(ForAll(test)) {
				out.LastShadowTri()=rays.lastShadowTri;
				return out;
			}
			out.LastShadowTri()=-1;
		}

		floatq tinv[3][size];
		for(int q=0;q<size;q++) {
			Vec3q idir=rays.IDir(q);
			tinv[0][q]=idir.x;
			tinv[1][q]=idir.y;
			tinv[2][q]=idir.z;
		}

		float sharedOrig[3];
		floatq torig[3][size];

		if(flags&isct::fShOrig) {
			sharedOrig[0]=rays.Origin(0).x[0];
			sharedOrig[1]=rays.Origin(0).y[0];
			sharedOrig[2]=rays.Origin(0).z[0];
		}
		else {
			for(int q=0;q<size;q++) {
				torig[0][q]=rays.Origin(q).x;
				torig[1][q]=rays.Origin(q).y;
				torig[2][q]=rays.Origin(q).z;
			}
		}

		floatq tMin[size],tMax[size];
		for(int q=0;q<size;q++) {
			tMin[q]=ConstEpsilon<floatq>();
			tMax[q]=1.0f/0.0f; //TODO: insert max distance
		}

		floatq fStackBegin[size*2*(maxLevel+2)],*fStack=fStackBegin;
		u32 nStackBegin[maxLevel+2],*nStack=nStackBegin;

		{
			Vec3p rMin=pMin,rMax=pMax;
			if(dirMask&1) Swap(rMin.x,rMax.x);
			if(dirMask&2) Swap(rMin.y,rMax.y);
			if(dirMask&4) Swap(rMin.z,rMax.z);

			Vec3q ttMin[size],ttMax[size];
			if(flags&isct::fShOrig) {
				Vec3q rrMin=Vec3q(rMin)-rays.Origin(0),rrMax=Vec3q(rMax)-rays.Origin(0);

				for(int q=0;q<size;q++) {
					ttMin[q]=rrMin*rays.IDir(q);
					ttMax[q]=rrMax*rays.IDir(q);
				}
			}
			else {
				for(int q=0;q<size;q++) {
					ttMin[q]=(Vec3q(rMin)-rays.Origin(q))*rays.IDir(q);
					ttMax[q]=(Vec3q(rMax)-rays.Origin(q))*rays.IDir(q);
				}
			}

			for(int q=0;q<size;q++) {
				ttMax[q].x=Min(ttMax[q].x,ttMax[q].y);
				tMax[q]=Min(tMax[q],ttMax[q].z);
				tMax[q]=Min(tMax[q],ttMax[q].x);
			}

			for(int q=0;q<size;q++) {
				ttMin[q].x=Max(ttMin[q].x,ttMin[q].y);
				tMin[q]=Max(tMin[q],ttMin[q].z);
				tMin[q]=Max(tMin[q],ttMin[q].x);
			}
		}
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
					Isct<f32x4,size,CElement::isctFlags|flags> tOut=elements[idx].Collide(rays);

					i32x4b fullMask(i32x4(0xffffffff).m);
					for(int q=0;q<size;q++) {
						i32x4b test=tOut.Distance(q)<out.Distance(q);
						out.Distance(q)=Min(out.Distance(q),tOut.Distance(q));

						if(flags&isct::fShadow) fullMask=fullMask&&test;
						else {
							out.Object(q)=Condition(test,i32x4(idx),out.Object(q));
							if(isctFlags&isct::fElement)
								out.Element(q)=Condition(test,tOut.Element(q),out.Element(q));
						}
					}
					if(flags&&isct::fShadow&&ForAll(fullMask))
						out.LastShadowTri()=idx;
				}

			POP_STACK:
				if(fStack==fStackBegin) break;

				fStack-=size*2;
				for(int q=0;q<size;q++) tMin[q]=fStack[q];
				for(int q=0;q<size;q++) tMax[q]=Min(fStack[size+q],out.Distance(q));
				
				--nStack;
				idx=*nStack;
				continue;
			}

			const Node *node=node0+(idx&Node::idxMask);

			int axis=node->Axis();
			int nidx=dirMask&(1<<axis)?1:0;

			floatq near[size],far[size];
			f32x4b test1,test2; {
				const floatq *inv=tinv[axis];

				if(flags&isct::fShOrig) {
					float tnear=node->clip[0]-sharedOrig[axis],tfar=node->clip[1]-sharedOrig[axis];
					if(nidx) Swap(tnear,tfar);
					
					near[0]=Min( floatq(tnear)*inv[0], tMax[0]);
					far [0]=Max( floatq(tfar)*inv[0], tMin[0]);
					test1=tMin[0]>near[0];
					test2=tMax[0]<far[0];
					
					for(int q=1;q<size;q++) {
						near[q]=Min( floatq(tnear)*inv[q], tMax[q]);
						far [q]=Max( floatq(tfar)*inv[q], tMin[q]);
						
						test1=test1&&tMin[q]>near[q];
						test2=test2&&tMax[q]<far [q];
					}
				}
				else {
					const floatq *start=torig[axis];
					float tnear=node->clip[0],tfar=node->clip[1];
					if(nidx) Swap(tnear,tfar);

					near[0]=Min( (floatq(tnear)-start[0])*inv[0], tMax[0]);
					far [0]=Max( (floatq(tfar) -start[0])*inv[0], tMin[0]);
					test1=tMin[0]>near[0];
					test2=tMax[0]<far[0];

					for(int q=1;q<size;q++) {
						near[q]=Min( (floatq(tnear)-start[q])*inv[q], tMax[q]);
						far [q]=Max( (floatq(tfar) -start[q])*inv[q], tMin[q]);
						
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
		
		return out;
	}

