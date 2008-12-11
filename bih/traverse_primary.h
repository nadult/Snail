
	template <int flags,int packetSize> Isct<f32x4,packetSize,isctFlags|flags>
		TraversePrimary(const RayGroup<packetSize,flags> &rays) const
	{
		if(!(flags&isct::fInvDir)) ThrowException("BIH::TraversePrimary: dir inverses must be avaliable");
		if(!(flags&isct::fShOrig)) ThrowException("BIH::TraversePrimary: origin must be shared");

		int dirMask=_mm_movemask_ps(_mm_shuffle_ps(_mm_shuffle_ps(rays.Dir(0).x.m,rays.Dir(0).y.m,0),
																	rays.Dir(0).z.m,0+(2<<2)))&7;
	
		Isct<f32x4,packetSize,isctFlags|flags> out;

		for(int q=0;q<packetSize;q++)
			out.Distance(q)=rays.MaxDist(q);

		TreeStats<1> &stats=out.Stats();
		stats.TracingPacket(4*packetSize);

		if(flags&isct::fShadow&&rays.lastShadowTri!=-1&&rays.lastShadowTri<elements.size()) {
		//	stats.Skip();
			f32x4b mask[packetSize];
			
			//TODO: if all rays hit, lastShadowTri=idx
			Isct<f32x4,packetSize,CElement::isctFlags|flags>
				tOut=elements[rays.lastShadowTri].Collide(rays);

			for(int q=0;q<packetSize;q++) {
				mask[q]=tOut.Distance(q)<out.Distance(q);
				out.Distance(q)=Condition(mask[q],floatq(0.0001f),out.Distance(q));
			}

			f32x4b test=mask[0];
			for(int q=1;q<packetSize;q++) test=test&&mask[q];
			
			if(ForAll(test)) {
				out.LastShadowTri()=rays.lastShadowTri;
				return out;
			}
			out.LastShadowTri()=-1;
		}

		float tMinInv[3],tMaxInv[3]; {
			floatq min[3]={1.0f/0.0f,1.0f/0.0f,1.0f/0.0f};
			floatq max[3]={-1.0f/0.0f,-1.0f/0.0f,-1.0f/0.0f};

			for(int q=0;q<packetSize;q++) {
				Vec3q inv=rays.IDir(q);
				min[0]=Min(min[0],inv.x); max[0]=Max(max[0],inv.x);
				min[1]=Min(min[1],inv.y); max[1]=Max(max[1],inv.y);
				min[2]=Min(min[2],inv.z); max[2]=Max(max[2],inv.z);
			}
			for(int n=0;n<3;n++) {
				tMinInv[n]=Minimize(min[n]);
				tMaxInv[n]=Maximize(max[n]);
			}
		}

		float sharedOrig[3];
		sharedOrig[0]=rays.Origin(0).x[0];
		sharedOrig[1]=rays.Origin(0).y[0];
		sharedOrig[2]=rays.Origin(0).z[0];

		float tMin=0.0f,tMax=1.0f/0.0f;

		float fStackBegin[2*(maxLevel+1)],*fStack=fStackBegin;
		u32 nStackBegin[maxLevel+1],*nStack=nStackBegin;

		{
			Vec3p origin(sharedOrig[0],sharedOrig[1],sharedOrig[2]);
			Vec3p ttMin=(pMin-origin)*Vec3p(tMinInv[0],tMinInv[1],tMinInv[2]);
			Vec3p ttMax=(pMax-origin)*Vec3p(tMaxInv[0],tMaxInv[1],tMaxInv[2]);
			if(dirMask&1) Swap(ttMin.x,ttMax.x);
			if(dirMask&2) Swap(ttMin.y,ttMax.y);
			if(dirMask&4) Swap(ttMin.z,ttMax.z);

			tMax=Min(Min(ttMax.x,ttMax.y),tMax);
			tMax=Min(ttMax.z,tMax);
			
			tMin=Max(Max(ttMin.x,ttMin.y),tMin);
			tMin=Max(ttMin.z,tMin);
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
					stats.Intersection(packetSize);

					Isct<f32x4,packetSize,CElement::isctFlags|flags> tOut=elements[idx].Collide(rays);

					i32x4b fullMask(i32x4(0xffffffff).m);
					for(int q=0;q<packetSize;q++) {
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

				fStack-=2;
				tMin=fStack[0];
				f32x4 tMin=out.Distance(0);
				for(int q=1;q<packetSize;q++) tMin=Max(tMin,out.Distance(q));
				tMax=Min(Maximize(tMin),fStack[1]);
				
				--nStack;
				idx=*nStack;
				continue;
			}

			const Node *node=node0+(idx&Node::idxMask);

			int axis=node->Axis();
			int nidx=dirMask&(1<<axis)?1:0;

			float near,far;
			{
				float tnear=node->clip[0]-sharedOrig[axis],tfar=node->clip[1]-sharedOrig[axis];
				float minInv=tMinInv[axis],maxInv=tMaxInv[axis];

				if(nidx) {
					Swap(tnear,tfar);
					near=Min(tnear*minInv,tMax);
					far =Max(tfar *maxInv,tMin); 
				}
				else {
					near=Min(tnear*maxInv,tMax);
					far =Max(tfar *minInv,tMin); 
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

			fStack[0]=far;
			fStack[1]=tMax;
			tMax=near;

			fStack+=2;

			*nStack=node->val[nidx^1];
			nStack++;
		
			idx=node->val[nidx];
		}
		
		return out;
	}

