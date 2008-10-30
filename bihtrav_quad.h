
	template <class Output>
	int BIHTree::TraverseQuad(const Vec3q &rOrigin,const Vec3q &tDir,Output output,int instanceId,
								int dirMask,int lastShadowTri) const {

		if(dirMask==8) {
			::Output<otNormal,float,u32> out((float*)output.dist,(u32*)output.object,(u32*)output.element,output.stats);
			
			TraverseMono(Vec3p(rOrigin.x[0],rOrigin.y[0],rOrigin.z[0]),Vec3p(tDir.x[0],tDir.y[0],tDir.z[0]),out,instanceId);
			out.dist++; out.object++; out.element++;
			TraverseMono(Vec3p(rOrigin.x[1],rOrigin.y[1],rOrigin.z[1]),Vec3p(tDir.x[1],tDir.y[1],tDir.z[1]),out,instanceId);
			out.dist++; out.object++; out.element++;
			TraverseMono(Vec3p(rOrigin.x[2],rOrigin.y[2],rOrigin.z[2]),Vec3p(tDir.x[2],tDir.y[2],tDir.z[2]),out,instanceId);
			out.dist++; out.object++; out.element++;
			TraverseMono(Vec3p(rOrigin.x[3],rOrigin.y[3],rOrigin.z[3]),Vec3p(tDir.x[3],tDir.y[3],tDir.z[3]),out,instanceId);
			return lastShadowTri;
		}

		floatq maxD=output.dist[0];
	

		TreeStats stats;
		stats.TracingRay(4);

		if(Output::type==otShadow) if(lastShadowTri!=-1&&gVals[0]) {
			floatq ret=objects[lastShadowTri].Collide(rOrigin,tDir);
			f32x4b mask=ret>0.0f&&ret<maxD;
			maxD=Condition(mask,0.0001f,maxD);
			stats.Intersection();

			if(ForAll(mask)) {
				stats.Skip();
				if(output.stats) output.stats->Update(stats);
				output.dist[0]=maxD;
				return lastShadowTri;
			}
			lastShadowTri=-1;
		}

		Vec3q invDir=VInv(Vec3q(tDir.x+0.000000000001f,tDir.y+0.000000000001f,tDir.z+0.000000000001f));
		floatq tinv[3]={invDir.x,invDir.y,invDir.z};
		floatq torig[3]={rOrigin.x,rOrigin.y,rOrigin.z};

		floatq minRet=maxD,tMin=ConstEpsilon<floatq>(),tMax=maxD;

		floatq fStackBegin[2*(maxLevel+2)],*fStack=fStackBegin;
		u32 stackBegin[maxLevel+2],*stack=stackBegin;

		tMax=Min(tMax,minRet);

		{
			Vec3q ttMin=(Vec3q(pMin)-rOrigin)*invDir;
			Vec3q ttMax=(Vec3q(pMax)-rOrigin)*invDir;
			if(dirMask&1) Swap(ttMin.x,ttMax.x);
			if(dirMask&2) Swap(ttMin.y,ttMax.y);
			if(dirMask&4) Swap(ttMin.z,ttMax.z);

			tMax=Min(Min(ttMax.x,ttMax.y),tMax);
			tMax=Min(ttMax.z,tMax);
			
			tMin=Max(Max(ttMin.x,ttMin.y),tMin);
			tMin=Max(ttMin.z,tMin);
		}


		const BIHNode *node0=&nodes[0];
		int idx=0;

		while(true) {
			stats.LoopIteration();
		
			if(idx&BIHNode::leafMask) {
				idx&=BIHNode::idxMask;

				{
					const BIHTriangle &obj=objects[idx];

					stats.Intersection();

					floatq ret=obj.Collide(rOrigin,tDir);
					f32x4b mask=ret<minRet&&ret>0.0f;

					if(ForAny(mask)) {
						minRet=Condition(mask,Output::type==otShadow?0.0001f:ret,minRet);
						if(Output::objectIndexes) {
							output.element[0]=Condition(i32x4b(mask),i32x4(idx),output.element[0]);
							output.object[0]=Condition(i32x4b(mask),i32x4(instanceId),output.object[0]);
						}

						if(Output::type==otShadow) if(ForAll(mask)) lastShadowTri=idx;
	
						tMax=Min(tMax,minRet);
					}
				}
				
			POP_STACK:
				if(stack==stackBegin) break;

				fStack-=2;
				tMin=fStack[0];
				tMax=Min(fStack[1],minRet);
				idx=*--stack;
				continue;
			}

			idx&=BIHNode::idxMask;
			const BIHNode *node=node0+idx;

			pattern.Touch(idx&BIHNode::idxMask,1);
			
			int axis=node->Axis();
			int sign=dirMask&(1<<axis)?1:0;
			floatq near,far; {
				floatq start=torig[axis],inv=tinv[axis];
				float tnear=node->ClipLeft(),tfar=node->ClipRight();
				if(sign) Swap(tnear,tfar);

				near=Min( (floatq(tnear)-start)*inv, tMax);
				far =Max( (floatq(tfar) -start)*inv, tMin);
			}

			if(ForAll(tMin>near)) {
				if(ForAll(tMax<far)) goto POP_STACK;

				// ARGH!!!!!!!!! this: tMin=far; is slower than:
				tMin=Max(tMin,far);
				idx=node->val[sign^1];
				continue;
			}
			if(ForAll(tMax<far)) {
				tMax=Min(tMax,near); //ARGH!
				idx=node->val[sign];
				continue;
			}

			fStack[0]=far;
			fStack[1]=tMax;
			fStack+=2;
			*stack++=node->val[sign^1];

			tMax=Min(tMax,near);//ARGH!
			idx=node->val[sign];
		}

		if(output.stats) output.stats->Update(stats);
		output.dist[0]=minRet;
		return lastShadowTri;
	}


