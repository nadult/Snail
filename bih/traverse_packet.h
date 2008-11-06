
namespace bih {

	template <class Element> template <int packetSize,bool sharedOrigin>
	typename Tree<Element>::template ReturnType<f32x4,packetSize>::Result Tree<Element>::TraversePacket(const RayGroup<packetSize,sharedOrigin,1> &rays,int dirMask) const {
		typename ReturnType<f32x4,packetSize>::Result out;
		for(int q=0;q<packetSize;q++) out.Distance(q)=1.0f/0.0f;

		enum { shared=sharedOrigin };

		TreeStats stats;
		stats.TracingPacket(4*packetSize);
		
		const Vec3q *rOrigin=&rays.Origin(0);
		const Vec3q *tDir=&rays.Dir(0);

	/*	if(outputType==otShadow) if(lastShadowTri!=-1&&gVals[0]) {
			floatq ret[packetSize];
			f32x4b mask[packetSize];
			
			const Element &element=elements[lastShadowTri];

			throw 0;
		//	for(int q=0;q<packetSize;q++) {
		//		ret[q]=element.Collide(rOrigin[q],tDir[q]);			
		//		mask[q]=ret[q]>0.0f&&ret[q]<out[q];
		//		out[q]=Condition(mask[q],floatq(0.0001f),out[q]);
		//	}

			f32x4b test=mask[0];
			for(int q=1;q<packetSize;q++) test=test&&mask[q];
			
			if(ForAll(test)) {
				stats.Skip();
				if(output.stats) output.stats->Update(stats);
				return lastShadowTri;
			}
			lastShadowTri=-1;
		} */

		const Vec3q *invDir=rays.idir;
		
		floatq tinv[3][packetSize];
		for(int q=0;q<packetSize;q++) {
			tinv[0][q]=invDir[q].x;
			tinv[1][q]=invDir[q].y;
			tinv[2][q]=invDir[q].z;
		}

		float sharedOrig[3];
		floatq torig[3][packetSize];

		if(sharedOrigin) {
			sharedOrig[0]=rOrigin[0].x[0];
			sharedOrig[1]=rOrigin[0].y[0];
			sharedOrig[2]=rOrigin[0].z[0];
		}
		else {
			for(int q=0;q<packetSize;q++) {
				torig[0][q]=rOrigin[q].x;
				torig[1][q]=rOrigin[q].y;
				torig[2][q]=rOrigin[q].z;
			}
		}

		floatq tMin[packetSize],tMax[packetSize];
		for(int q=0;q<packetSize;q++) {
			tMin[q]=ConstEpsilon<floatq>();
			tMax[q]=1.0f/0.0f; //TODO: insert max distance
		}

		floatq fStackBegin[packetSize*2*(maxLevel+2)],*fStack=fStackBegin;
		u32 nStackBegin[maxLevel+2],*nStack=nStackBegin;

		{
			Vec3p rMin=pMin,rMax=pMax;
			if(dirMask&1) Swap(rMin.x,rMax.x);
			if(dirMask&2) Swap(rMin.y,rMax.y);
			if(dirMask&4) Swap(rMin.z,rMax.z);

			Vec3q ttMin[packetSize],ttMax[packetSize];
			if(sharedOrigin) {
				Vec3q rrMin=Vec3q(rMin)-rOrigin[0],rrMax=Vec3q(rMax)-rOrigin[0];

				for(int q=0;q<packetSize;q++) {
					ttMin[q]=rrMin*invDir[q];
					ttMax[q]=rrMax*invDir[q];
				}
			}
			else {
				for(int q=0;q<packetSize;q++) {
					ttMin[q]=(Vec3q(rMin)-rOrigin[q])*invDir[q];
					ttMax[q]=(Vec3q(rMax)-rOrigin[q])*invDir[q];
				}
			}

			for(int q=0;q<packetSize;q++) {
				ttMax[q].x=Min(ttMax[q].x,ttMax[q].y);
				tMax[q]=Min(tMax[q],ttMax[q].z);
				tMax[q]=Min(tMax[q],ttMax[q].x);
			}

			for(int q=0;q<packetSize;q++) {
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
					stats.Intersection(packetSize);

					const Element &element=elements[idx];
					
					stats.Intersection(packetSize);

					//todo: if all rays hit, lastShadowTri=idx
					typename ReturnType<f32x4,packetSize>::BaseIntersection tOut=element.Collide(rays);
					for(int q=0;q<packetSize;q++) {
						i32x4b test=tOut.Distance(q)<out.Distance(q);
						out.Distance(q)=Min(out.Distance(q),tOut.Distance(q));
						out.Object(q)=Condition(test,i32x4(idx),out.Object(q));
						if(ReturnType<f32x4,packetSize>::Result::flags&ifElement)
							out.Element(q)=Condition(test,tOut.Element(q),out.Element(q));
					}
				}

			POP_STACK:
				if(fStack==fStackBegin) break;

				fStack-=packetSize*2;
				for(int q=0;q<packetSize;q++) tMin[q]=fStack[q];
				for(int q=0;q<packetSize;q++) tMax[q]=Min(fStack[packetSize+q],out.Distance(q)); //TODO: max dist
				
				--nStack;
				idx=*nStack;
				continue;
			}

			const Node *node=node0+(idx&Node::idxMask);

			int axis=node->Axis();
			int nidx=dirMask&(1<<axis)?1:0;

			floatq near[packetSize],far[packetSize];
			f32x4b test1,test2; {
				const floatq *inv=tinv[axis];

				if(sharedOrigin) {
					float tnear=node->clip[0]-sharedOrig[axis],tfar=node->clip[1]-sharedOrig[axis];
					if(nidx) Swap(tnear,tfar);
					
					near[0]=Min( floatq(tnear)*inv[0], tMax[0]);
					far [0]=Max( floatq(tfar)*inv[0], tMin[0]);
					test1=tMin[0]>near[0];
					test2=tMax[0]<far[0];
					
					for(int q=1;q<packetSize;q++) {
						near[q]=Min( floatq(tnear)*inv[q], tMax[q]);
						far [q]=Max( floatq(tfar)*inv[q], tMin[q]);
						
						test1=test1&&tMin[q]>near[q];
						test2=test2&&tMax[q]<far [q];
					}
				}
				else {
					floatq *start=torig[axis];
					float tnear=node->clip[0],tfar=node->clip[1];
					if(nidx) Swap(tnear,tfar);

					near[0]=Min( (floatq(tnear)-start[0])*inv[0], tMax[0]);
					far [0]=Max( (floatq(tfar) -start[0])*inv[0], tMin[0]);
					test1=tMin[0]>near[0];
					test2=tMax[0]<far[0];

					for(int q=1;q<packetSize;q++) {
						near[q]=Min( (floatq(tnear)-start[q])*inv[q], tMax[q]);
						far [q]=Max( (floatq(tfar) -start[q])*inv[q], tMin[q]);
						
						test1=test1&&tMin[q]>near[q];
						test2=test2&&tMax[q]<far [q];
					}
				}
			}

			if(ForAll(test1)) {
				if(ForAll(test2)) goto POP_STACK;

				if(packetSize==1) tMin[0]=Max(tMin[0],far[0]);
				else for(int q=0;q<packetSize;q++) tMin[q]=far[q];
				idx=node->val[nidx^1];
				continue;
			}
			if(ForAll(test2)) {
				if(packetSize==1) tMax[0]=Min(tMax[0],near[0]);
				else for(int q=0;q<packetSize;q++) tMax[q]=near[q];
				idx=node->val[nidx];
				continue;
			}

			for(int q=0;q<packetSize;q++) fStack[q]=far[q];
			for(int q=0;q<packetSize;q++) {
				fStack[packetSize+q]=tMax[q];
				tMax[q]=near[q];
			}
			fStack+=packetSize*2;

			*nStack=node->val[nidx^1];
			nStack++;
		
			idx=node->val[nidx];
		}

	//	if(sizeof(Element)==64&&output.stats) output.stats->Update(stats);
		
		return out;
	}

}

