
	template <class AccStruct> template <class Output>
	void BIHTree<AccStruct>::TraverseQuad(const Vec3q &rOrigin,const Vec3q &tDir,Output output,int dirMask) const {
		floatq maxD=output.dist[0];

		TreeStats stats;
		stats.TracingPacket(4);

		Vec3q invDir=VInv(Vec3q(tDir.x+0.000000000001f,tDir.y+0.000000000001f,tDir.z+0.000000000001f));
		floatq tinv[3]={invDir.x,invDir.y,invDir.z};
		floatq torig[3]={rOrigin.x,rOrigin.y,rOrigin.z};

		int dSign[3]; FillDSignArray(dirMask,dSign);
		floatq minRet=maxD,tMin=ConstEpsilon<floatq>(),tMax=maxD;

		floatq fStackBegin[2*(maxLevel+2)],*fStack=fStackBegin;
		u32 stackBegin[maxLevel+2],*stack=stackBegin;

		tMax=Min(tMax,minRet);

		{
			Vec3q ttMin=(Vec3q(pMin)-rOrigin)*invDir;
			Vec3q ttMax=(Vec3q(pMax)-rOrigin)*invDir;
			if(dSign[0]) Swap(ttMin.x,ttMax.x);
			if(dSign[1]) Swap(ttMin.y,ttMax.y);
			if(dSign[2]) Swap(ttMin.z,ttMax.z);

			tMax=Min(Min(ttMax.x,ttMax.y),tMax);
			tMax=Min(ttMax.z,tMax);
			
			tMin=Max(Max(ttMin.x,ttMin.y),tMin);
			tMin=Max(ttMin.z,tMin);
		}

		const BIHNode *node0=&nodes[0],*node;
		int idx=0;

		while(true) {
			stats.LoopIteration();
		
			if(idx&BIHNode::leafMask) {
				idx&=BIHNode::idxMask;

				{
					stats.Intersection();
					const Object &obj=objects[idx];
					floatq ret=obj.Collide(rOrigin,tDir);
					f32x4b mask=ret<minRet&&ret>0.0f;

					if(ForAny(mask)) {
						minRet=Condition(mask,ret,minRet);
						if(Output::objectIndexes)
							output.object[0]=Condition(i32x4b(mask),i32x4(idx),output.object[0]);
	
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

			node=node0+(idx&BIHNode::idxMask);
			int axis=node->Axis();
			int sign=dSign[axis];
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

		output.stats->Update(stats);
		output.dist[0]=minRet;
	}


