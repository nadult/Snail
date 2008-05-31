
	inline float Maximize(const floatq &t) { return Max(Max(t[0],t[1]),Max(t[2],t[3])); }
	inline float Minimize(const floatq &t) { return Min(Min(t[0],t[1]),Min(t[2],t[3])); }
	inline Vec3p Maximize(const Vec3q &v) { return Vec3p(Maximize(v.x),Maximize(v.y),Maximize(v.z)); }
	inline Vec3p Minimize(const Vec3q &v) { return Vec3p(Minimize(v.x),Minimize(v.y),Minimize(v.z)); }

	template <class AccStruct> template <class Output>
	void BIHTree<AccStruct>:: TraverseQuad4Primary(const Vec3q *rOrigin,const Vec3q *tDir,floatq *out,i32x4 *object,TreeStats *tstats,int dirMask) const {
		TreeStats stats;
		stats.TracingPacket(16);

		Vec3p tCornerOrig=Vec3p(rOrigin[0].x[0],rOrigin[0].y[0],rOrigin[0].z[0]);
		int dSign[3]; FillDSignArray(dirMask,dSign);

		Vec3p cMaxInv,cMinInv; {
			Vec3q aDir[4]={VAbs(tDir[0]),VAbs(tDir[1]),VAbs(tDir[2]),VAbs(tDir[3])};
			Vec3p tMax=Maximize( VMax(VMax(aDir[0],aDir[1]),VMax(aDir[2],aDir[3])) );
			Vec3p tMin=Minimize( VMin(VMin(aDir[0],aDir[1]),VMin(aDir[2],aDir[3])) );
			Vec3p shift(0.000000000001f,0.000000000001f,0.000000000001f);
			(&tMin.x)[3]=(&tMax.x)[3]=1.0f;
			tMin=VInv(tMin+shift);
			tMax=VInv(tMax+shift);
			if(dSign[0]) { tMin.x=-tMin.x; tMax.x=-tMax.x; }
			if(dSign[1]) { tMin.y=-tMin.y; tMax.y=-tMax.y; }
			if(dSign[2]) { tMin.z=-tMin.z; tMax.z=-tMax.z; }
			cMaxInv=VMax(tMin,tMax);
			cMinInv=VMin(tMin,tMax);
		}

		floatq minRet[4]={out[0],out[1],out[2],out[3]};
		float cMaxMax=Maximize( Max(Max(minRet[0],minRet[1]),Max(minRet[2],minRet[3])) );

		float fStackBegin[2*(maxLevel+2)],*fStack=fStackBegin;
		u32 nStackBegin[maxLevel+2],*nStack=nStackBegin;

		float cMax,cMin; {
			Vec3p rMin=pMin,rMax=pMax;
			if(dSign[0]) Swap(rMin.x,rMax.x);
			if(dSign[1]) Swap(rMin.y,rMax.y);
			if(dSign[2]) Swap(rMin.z,rMax.z);

			Vec3p ttMin=(rMin-tCornerOrig)*cMaxInv;
			Vec3p ttMax=(rMax-tCornerOrig)*cMinInv;

			cMin=Max(Max(ConstEpsilon<float>(),ttMin.x),Max(ttMin.y,ttMin.z));
			cMax=Min(Min(cMaxMax			  ,ttMax.x),Min(ttMax.y,ttMax.z));
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
					floatq u[4],v[4],val; {
						Vec3q tvec=rOrigin[0]-Vec3q(obj.a);
						Vec3q ba(obj.ba.x,obj.ba.y,obj.ba.z),ca(obj.ca.x,obj.ca.y,obj.ca.z);
						Vec3q cross1=ba^tvec,cross2=tvec^ca;

						u[0]=tDir[0]|cross1; v[0]=tDir[0]|cross2;
						u[1]=tDir[1]|cross1; v[1]=tDir[1]|cross2;
						u[2]=tDir[2]|cross1; v[2]=tDir[2]|cross2;
						u[3]=tDir[3]|cross1; v[3]=tDir[3]|cross2;

						val=-(tvec|nrm);
					}

					floatq nrmLen=floatq( ((float*)&obj.ca)[3] );
					{
						floatq det=tDir[0]|nrm;
						f32x4b mask=Min(u[0],v[0])>=0.0f&&u[0]+v[0]<=det*nrmLen;
						if(ForAny(mask)) {
							floatq dist=Condition(mask,val/det,minRet[0]);
							mask=dist<minRet[0]&&dist>0.0f;
							minRet[0]=Condition(mask,dist,minRet[0]);
							if(Output::objectIndexes)
								object[0]=Condition(i32x4b(mask),i32x4(idx),object[0]);
							stats.IntersectPass();
						} else stats.IntersectFail();
					} {
						floatq det=tDir[1]|nrm;
						f32x4b mask=Min(u[1],v[1])>=0.0f&&u[1]+v[1]<=det*nrmLen;
						if(ForAny(mask)) {
							floatq dist=Condition(mask,val/det,minRet[1]);
							mask=dist<minRet[1]&&dist>0.0f;
							minRet[1]=Condition(mask,dist,minRet[1]);
							if(Output::objectIndexes)
								object[1]=Condition(i32x4b(mask),i32x4(idx),object[1]);
							stats.IntersectPass();
						} else stats.IntersectFail();
					} {
						floatq det=tDir[2]|nrm;
						f32x4b mask=Min(u[2],v[2])>=0.0f&&u[2]+v[2]<=det*nrmLen;
						if(ForAny(mask)) {
							floatq dist=Condition(mask,val/det,minRet[2]);
							mask=dist<minRet[2]&&dist>0.0f;
							minRet[2]=Condition(mask,dist,minRet[2]);
							if(Output::objectIndexes)
								object[2]=Condition(i32x4b(mask),i32x4(idx),object[2]);
							stats.IntersectPass();
						} else stats.IntersectFail();
					} {
						floatq det=tDir[3]|nrm;
						f32x4b mask=Min(u[3],v[3])>=0.0f&&u[3]+v[3]<=det*nrmLen;
						if(ForAny(mask)) {
							floatq dist=Condition(mask,val/det,minRet[3]);
							mask=dist<minRet[3]&&dist>0.0f;
							minRet[3]=Condition(mask,dist,minRet[3]);
							if(Output::objectIndexes)
								object[3]=Condition(i32x4b(mask),i32x4(idx),object[3]);
							stats.IntersectPass();
						} else stats.IntersectFail();
					}
					cMaxMax=Maximize( Max(Max(minRet[0],minRet[1]),Max(minRet[2],minRet[3])) );
				}

			POP_STACK:
				if(fStack==fStackBegin) break;

				fStack-=2;
				cMin=fStack[0];
				cMax=Min(cMaxMax,fStack[1]);

				--nStack;
				idx=*nStack;
				continue;
			}

			const BIHNode *node=node0+(idx&BIHNode::idxMask);
			int axis=node->Axis();
			int nidx=dSign[axis];

			float cNear,cFar; {
				float tnear=node->ClipLeft()-(&tCornerOrig.x)[axis];
				float tfar=node->ClipRight()-(&tCornerOrig.x)[axis];
				if(nidx) Swap(tnear,tfar);

				cNear=(tnear<0?(&cMinInv.x)[axis]:(&cMaxInv.x)[axis])*tnear;
				cFar =(tfar <0?(&cMaxInv.x)[axis]:(&cMinInv.x)[axis])*tfar;

				cNear=Min(cNear, cMax);
				cFar =Max(cFar , cMin);
			}
		
			if(cMin>cNear) {
				if(cMax<cFar) goto POP_STACK;

				cMin=cFar;
				idx=node->val[nidx^1];
				continue;
			}
			if(cMax<cFar) {
				cMax=cNear;
				idx=node->val[nidx];
				continue;
			}

			
			fStack[0]=cFar;
			fStack[1]=cMax;
			fStack+=2;

			*nStack=node->val[nidx^1];
			nStack++;

			cMax=cNear;
			idx=node->val[nidx];
		}

		if(tstats) tstats->Update(stats);
		out[0]=minRet[0];
		out[1]=minRet[1];
		out[2]=minRet[2];
		out[3]=minRet[3];
	}

