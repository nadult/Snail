
	template <class AccStruct> template <class Output,bool sharedOrigin>
	void BIHTree<AccStruct>:: TraverseQuad4(const Vec3q *rOrigin,const Vec3q *tDir,floatq *out,i32x4 *object,TreeStats *tstats,int dirMask) const {
		floatq maxD[4]={out[0],out[1],out[2],out[3]};

		TreeStats stats;
		stats.TracingPacket(16);

		Vec3q invDir[4]={
			VInv(Vec3q(tDir[0].x+0.000000000001f,tDir[0].y+0.000000000001f,tDir[0].z+0.000000000001f)),
			VInv(Vec3q(tDir[1].x+0.000000000001f,tDir[1].y+0.000000000001f,tDir[1].z+0.000000000001f)),
			VInv(Vec3q(tDir[2].x+0.000000000001f,tDir[2].y+0.000000000001f,tDir[2].z+0.000000000001f)),
			VInv(Vec3q(tDir[3].x+0.000000000001f,tDir[3].y+0.000000000001f,tDir[3].z+0.000000000001f)),
		};
		floatq tinv[3][4]={
			{invDir[0].x,invDir[1].x,invDir[2].x,invDir[3].x},
			{invDir[0].y,invDir[1].y,invDir[2].y,invDir[3].y},
			{invDir[0].z,invDir[1].z,invDir[2].z,invDir[3].z} };
		floatq torig[3][4]={
			{rOrigin[0].x,rOrigin[1].x,rOrigin[2].x,rOrigin[3].x},
			{rOrigin[0].y,rOrigin[1].y,rOrigin[2].y,rOrigin[3].y},
			{rOrigin[0].z,rOrigin[1].z,rOrigin[2].z,rOrigin[3].z} };

		floatq minRet[4]={maxD[0],maxD[1],maxD[2],maxD[3]};

		floatq tMin[4],tMax[4];
		tMin[0]=tMin[1]=tMin[2]=tMin[3]=ConstEpsilon<floatq>();
		tMax[0]=Min(maxD[0],minRet[0]);
		tMax[1]=Min(maxD[1],minRet[1]);
		tMax[2]=Min(maxD[2],minRet[2]);
		tMax[3]=Min(maxD[3],minRet[3]);

		floatq fStackBegin[8*(maxLevel+2)],*fStack=fStackBegin;
		u32 nStackBegin[maxLevel+2],*nStack=nStackBegin;

		{
			Vec3p rMin=pMin,rMax=pMax;
			if(dirMask&1) Swap(rMin.x,rMax.x);
			if(dirMask&2) Swap(rMin.y,rMax.y);
			if(dirMask&4) Swap(rMin.z,rMax.z);

			Vec3q ttMin[4]={
				(Vec3q(rMin)-rOrigin[0])*invDir[0],
				(Vec3q(rMin)-rOrigin[1])*invDir[1],
				(Vec3q(rMin)-rOrigin[2])*invDir[2],
				(Vec3q(rMin)-rOrigin[3])*invDir[3],
			}; Vec3q ttMax[4]={
				(Vec3q(rMax)-rOrigin[0])*invDir[0],
				(Vec3q(rMax)-rOrigin[1])*invDir[1],
				(Vec3q(rMax)-rOrigin[2])*invDir[2],
				(Vec3q(rMax)-rOrigin[3])*invDir[3],
			};

			ttMax[0].x=Min(ttMax[0].x,ttMax[0].y);
			ttMax[1].x=Min(ttMax[1].x,ttMax[1].y);
			ttMax[2].x=Min(ttMax[2].x,ttMax[2].y);
			ttMax[3].x=Min(ttMax[3].x,ttMax[3].y);

			ttMin[0].x=Max(ttMin[0].x,ttMin[0].y);
			ttMin[1].x=Max(ttMin[1].x,ttMin[1].y);
			ttMin[2].x=Max(ttMin[2].x,ttMin[2].y);
			ttMin[3].x=Max(ttMin[3].x,ttMin[3].y);

			tMax[0]=Min(tMax[0],ttMax[0].z); tMax[0]=Min(tMax[0],ttMax[0].x);
			tMax[1]=Min(tMax[1],ttMax[1].z); tMax[1]=Min(tMax[1],ttMax[1].x);
			tMax[2]=Min(tMax[2],ttMax[2].z); tMax[2]=Min(tMax[2],ttMax[2].x);
			tMax[3]=Min(tMax[3],ttMax[3].z); tMax[3]=Min(tMax[3],ttMax[3].x);
			
			tMin[0]=Max(tMin[0],ttMin[0].z); tMin[0]=Max(tMin[0],ttMin[0].x);
			tMin[1]=Max(tMin[1],ttMin[1].z); tMin[1]=Max(tMin[1],ttMin[1].x);
			tMin[2]=Max(tMin[2],ttMin[2].z); tMin[2]=Max(tMin[2],ttMin[2].x);
			tMin[3]=Max(tMin[3],ttMin[3].z); tMin[3]=Max(tMin[3],ttMin[3].x);
		}
		ObjectIdxBuffer<4> mailbox;

		const BIHNode *node0=&nodes[0];
		int idx=0;

		while(true) {
			stats.LoopIteration();

			if(idx&BIHNode::leafMask) {
				idx&=BIHNode::idxMask;

				if(!mailbox.Find(idx)) {
					mailbox.Insert(idx);
					stats.Intersection(4);

					const Object &obj=objects[idx];

					Vec3p nrm=obj.Nrm();
					floatq u[4],v[4],val[4];
					if(sharedOrigin) {
						Vec3q tvec=rOrigin[0]-Vec3q(obj.a);
						Vec3q ba(obj.ba.x,obj.ba.y,obj.ba.z),ca(obj.ca.x,obj.ca.y,obj.ca.z);
						Vec3q cross1=ba^tvec,cross2=tvec^ca;

						u[0]=tDir[0]|cross1; v[0]=tDir[0]|cross2;
						u[1]=tDir[1]|cross1; v[1]=tDir[1]|cross2;
						u[2]=tDir[2]|cross1; v[2]=tDir[2]|cross2;
						u[3]=tDir[3]|cross1; v[3]=tDir[3]|cross2;

						val[0]=-(tvec|nrm);
						val[1]=val[2]=val[3]=val[0];
					}
					else {
						Vec3q tvec[4]; {
							Vec3q a(obj.a.x,obj.a.y,obj.a.z);
							tvec[0]=rOrigin[0]-a;
							tvec[1]=rOrigin[1]-a;
							tvec[2]=rOrigin[2]-a;
							tvec[3]=rOrigin[3]-a;
						}
						val[0]=-(tvec[0]|nrm);
						val[1]=-(tvec[1]|nrm);
						val[2]=-(tvec[2]|nrm);
						val[3]=-(tvec[3]|nrm);

						Vec3q ba(obj.ba.x,obj.ba.y,obj.ba.z),ca(obj.ca.x,obj.ca.y,obj.ca.z);
						u[0]=tDir[0]|(ba^tvec[0]); v[0]=tDir[0]|(tvec[0]^ca);
						u[1]=tDir[1]|(ba^tvec[1]); v[1]=tDir[1]|(tvec[1]^ca);
						u[2]=tDir[2]|(ba^tvec[2]); v[2]=tDir[2]|(tvec[2]^ca);
						u[3]=tDir[3]|(ba^tvec[3]); v[3]=tDir[3]|(tvec[3]^ca);
					}
				
					floatq nrmLen=floatq( ((float*)&obj.ca)[3] );
					{
						floatq det=tDir[0]|nrm;
						f32x4b mask=Min(u[0],v[0])>=0.0f&&u[0]+v[0]<=det*nrmLen;
						if(ForAny(mask)) {
							floatq dist=Condition(mask,val[0]/det,minRet[0]);
							mask=dist<minRet[0]&&dist>0.0f;
							minRet[0]=Condition(mask,Output::type==otShadow?0.00001f:dist,minRet[0]);
							if(Output::objectIndexes)
								object[0]=Condition(i32x4b(mask),i32x4(idx),object[0]);
							stats.IntersectPass();
						} else stats.IntersectFail();
					} {
						floatq det=tDir[1]|nrm;
						f32x4b mask=Min(u[1],v[1])>=0.0f&&u[1]+v[1]<=det*nrmLen;
						if(ForAny(mask)) {
							floatq dist=Condition(mask,val[1]/det,minRet[1]);
							mask=dist<minRet[1]&&dist>0.0f;
							minRet[1]=Condition(mask,Output::type==otShadow?0.00001f:dist,minRet[1]);
							if(Output::objectIndexes)
								object[1]=Condition(i32x4b(mask),i32x4(idx),object[1]);
							stats.IntersectPass();
						} else stats.IntersectFail();
					} {
						floatq det=tDir[2]|nrm;
						f32x4b mask=Min(u[2],v[2])>=0.0f&&u[2]+v[2]<=det*nrmLen;
						if(ForAny(mask)) {
							floatq dist=Condition(mask,val[2]/det,minRet[2]);
							mask=dist<minRet[2]&&dist>0.0f;
							minRet[2]=Condition(mask,Output::type==otShadow?0.00001f:dist,minRet[2]);
							if(Output::objectIndexes)
								object[2]=Condition(i32x4b(mask),i32x4(idx),object[2]);
							stats.IntersectPass();
						} else stats.IntersectFail();
					} {
						floatq det=tDir[3]|nrm;
						f32x4b mask=Min(u[3],v[3])>=0.0f&&u[3]+v[3]<=det*nrmLen;
						if(ForAny(mask)) {
							floatq dist=Condition(mask,val[3]/det,minRet[3]);
							mask=dist<minRet[3]&&dist>0.0f;
							minRet[3]=Condition(mask,Output::type==otShadow?0.00001f:dist,minRet[3]);
							if(Output::objectIndexes)
								object[3]=Condition(i32x4b(mask),i32x4(idx),object[3]);
							stats.IntersectPass();
						} else stats.IntersectFail();
					}
				}

			POP_STACK:
				if(fStack==fStackBegin) break;

				fStack-=8;
				tMin[0]=fStack[0];
				tMin[1]=fStack[1];
				tMin[2]=fStack[2];
				tMin[3]=fStack[3];
				tMax[0]=Min(fStack[4],minRet[0]);
				tMax[1]=Min(fStack[5],minRet[1]);
				tMax[2]=Min(fStack[6],minRet[2]);
				tMax[3]=Min(fStack[7],minRet[3]);
				--nStack;
				idx=*nStack;
				continue;
			}

			const BIHNode *node=node0+(idx&BIHNode::idxMask);
			int axis=node->Axis();
			int nidx=dirMask&(1<<axis)?1:0;

			floatq near[4],far[4]; {
				floatq *start=torig[axis],*inv=tinv[axis];
				float tnear=node->ClipLeft(),tfar=node->ClipRight();
				if(nidx) Swap(tnear,tfar);

				near[0]=Min( (floatq(tnear)-start[0])*inv[0], tMax[0]);
				near[1]=Min( (floatq(tnear)-start[1])*inv[1], tMax[1]);
				near[2]=Min( (floatq(tnear)-start[2])*inv[2], tMax[2]);
				near[3]=Min( (floatq(tnear)-start[3])*inv[3], tMax[3]);

				far [0]=Max( (floatq(tfar) -start[0])*inv[0], tMin[0]);
				far [1]=Max( (floatq(tfar) -start[1])*inv[1], tMin[1]);
				far [2]=Max( (floatq(tfar) -start[2])*inv[2], tMin[2]);
				far [3]=Max( (floatq(tfar) -start[3])*inv[3], tMin[3]);
			}

			f32x4b test1=tMin[0]>near[0]&&tMin[1]>near[1]&&tMin[2]>near[2]&&tMin[3]>near[3];
			f32x4b test2=tMax[0]<far [0]&&tMax[1]<far [1]&&tMax[2]<far [2]&&tMax[3]<far [3];

			if(ForAll(test1)) {
				if(ForAll(test2)) goto POP_STACK;

				tMin[0]=far[0];
				tMin[1]=far[1];
				tMin[2]=far[2];
				tMin[3]=far[3];
				idx=node->val[nidx^1];
				continue;
			}
			if(ForAll(test2)) {
				tMax[0]=near[0];
				tMax[1]=near[1];
				tMax[2]=near[2];
				tMax[3]=near[3];
				idx=node->val[nidx];
				continue;
			}

			fStack[0]=far[0];
			fStack[1]=far[1];
			fStack[2]=far[2];
			fStack[3]=far[3];
			fStack[4]=tMax[0];
			fStack[5]=tMax[1];
			fStack[6]=tMax[2];
			fStack[7]=tMax[3];
			fStack+=8;

			*nStack=node->val[nidx^1];
			nStack++;

			tMax[0]=near[0];
			tMax[1]=near[1];
			tMax[2]=near[2];
			tMax[3]=near[3];
			
			idx=node->val[nidx];
		}

		if(tstats) tstats->Update(stats);
		out[0]=minRet[0];
		out[1]=minRet[1];
		out[2]=minRet[2];
		out[3]=minRet[3];
	}

