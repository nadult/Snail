
	template <class Output,bool sharedOrigin>
	void BIHTree:: TraverseQuad4(const Vec3q *rOrigin,const Vec3q *tDir,floatq *out,i32x4 *object,TreeStats *tstats,int dirMask) const {
		TreeStats stats;
		stats.TracingPacket(16);

		const Vec3q invDir[4]={
			VInv(Vec3q(tDir[0].x+0.000000000001f,tDir[0].y+0.000000000001f,tDir[0].z+0.000000000001f)),
			VInv(Vec3q(tDir[1].x+0.000000000001f,tDir[1].y+0.000000000001f,tDir[1].z+0.000000000001f)),
			VInv(Vec3q(tDir[2].x+0.000000000001f,tDir[2].y+0.000000000001f,tDir[2].z+0.000000000001f)),
			VInv(Vec3q(tDir[3].x+0.000000000001f,tDir[3].y+0.000000000001f,tDir[3].z+0.000000000001f)),
		};
		const floatq tinv[3][4]={
			{invDir[0].x,invDir[1].x,invDir[2].x,invDir[3].x},
			{invDir[0].y,invDir[1].y,invDir[2].y,invDir[3].y},
			{invDir[0].z,invDir[1].z,invDir[2].z,invDir[3].z} };

		float sharedOrig[3];
		floatq torig[3][4];

		if(sharedOrigin) {
			sharedOrig[0]=rOrigin[0].x[0];
			sharedOrig[1]=rOrigin[0].y[0];
			sharedOrig[2]=rOrigin[0].z[0];
		}
		else {
			torig[0][0]=rOrigin[0].x; torig[0][1]=rOrigin[1].x; torig[0][2]=rOrigin[2].x; torig[0][3]=rOrigin[3].x;
			torig[1][0]=rOrigin[0].y; torig[1][1]=rOrigin[1].y; torig[1][2]=rOrigin[2].y; torig[1][3]=rOrigin[3].y;
			torig[2][0]=rOrigin[0].z; torig[2][1]=rOrigin[1].z; torig[2][2]=rOrigin[2].z; torig[2][3]=rOrigin[3].z;
		}

		floatq tMin[4],tMax[4];
		tMin[0]=tMin[1]=tMin[2]=tMin[3]=ConstEpsilon<floatq>();
		tMax[0]=out[0];
		tMax[1]=out[1];
		tMax[2]=out[2];
		tMax[3]=out[3];

		floatq fStackBegin[8*(maxLevel+2)],*fStack=fStackBegin;
		u32 nStackBegin[maxLevel+2],*nStack=nStackBegin;

		{
			Vec3p rMin=pMin,rMax=pMax;
			if(dirMask&1) Swap(rMin.x,rMax.x);
			if(dirMask&2) Swap(rMin.y,rMax.y);
			if(dirMask&4) Swap(rMin.z,rMax.z);

			Vec3q ttMin[4],ttMax[4];
			if(sharedOrigin) {
				Vec3q rrMin=Vec3q(rMin)-rOrigin[0],rrMax=Vec3q(rMax)-rOrigin[0];

				ttMin[0]=rrMin*invDir[0];
				ttMin[1]=rrMin*invDir[1];
				ttMin[2]=rrMin*invDir[2];
				ttMin[3]=rrMin*invDir[3];
				ttMax[0]=rrMax*invDir[0];
				ttMax[1]=rrMax*invDir[1];
				ttMax[2]=rrMax*invDir[2];
				ttMax[3]=rrMax*invDir[3];
			}
			else {
				ttMin[0]=(Vec3q(rMin)-rOrigin[0])*invDir[0];
				ttMin[1]=(Vec3q(rMin)-rOrigin[1])*invDir[1];
				ttMin[2]=(Vec3q(rMin)-rOrigin[2])*invDir[2];
				ttMin[3]=(Vec3q(rMin)-rOrigin[3])*invDir[3];
				ttMax[0]=(Vec3q(rMax)-rOrigin[0])*invDir[0];
				ttMax[1]=(Vec3q(rMax)-rOrigin[1])*invDir[1];
				ttMax[2]=(Vec3q(rMax)-rOrigin[2])*invDir[2];
				ttMax[3]=(Vec3q(rMax)-rOrigin[3])*invDir[3];
			}

			ttMax[0].x=Min(ttMax[0].x,ttMax[0].y); tMax[0]=Min(tMax[0],ttMax[0].z); tMax[0]=Min(tMax[0],ttMax[0].x);
			ttMax[1].x=Min(ttMax[1].x,ttMax[1].y); tMax[1]=Min(tMax[1],ttMax[1].z); tMax[1]=Min(tMax[1],ttMax[1].x);
			ttMax[2].x=Min(ttMax[2].x,ttMax[2].y); tMax[2]=Min(tMax[2],ttMax[2].z); tMax[2]=Min(tMax[2],ttMax[2].x);
			ttMax[3].x=Min(ttMax[3].x,ttMax[3].y); tMax[3]=Min(tMax[3],ttMax[3].z); tMax[3]=Min(tMax[3],ttMax[3].x);

			ttMin[0].x=Max(ttMin[0].x,ttMin[0].y); tMin[0]=Max(tMin[0],ttMin[0].z); tMin[0]=Max(tMin[0],ttMin[0].x);
			ttMin[1].x=Max(ttMin[1].x,ttMin[1].y); tMin[1]=Max(tMin[1],ttMin[1].z); tMin[1]=Max(tMin[1],ttMin[1].x);
			ttMin[2].x=Max(ttMin[2].x,ttMin[2].y); tMin[2]=Max(tMin[2],ttMin[2].z); tMin[2]=Max(tMin[2],ttMin[2].x);
			ttMin[3].x=Max(ttMin[3].x,ttMin[3].y); tMin[3]=Max(tMin[3],ttMin[3].z); tMin[3]=Max(tMin[3],ttMin[3].x);
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

					const BIHTriangle &obj=objects[idx];

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
						val[0]=(tvec[0]|-nrm);
						val[1]=(tvec[1]|-nrm);
						val[2]=(tvec[2]|-nrm);
						val[3]=(tvec[3]|-nrm);

						Vec3q ba(obj.ba.x,obj.ba.y,obj.ba.z),ca(obj.ca.x,obj.ca.y,obj.ca.z);
						u[0]=tDir[0]|(ba^tvec[0]); v[0]=tDir[0]|(tvec[0]^ca);
						u[1]=tDir[1]|(ba^tvec[1]); v[1]=tDir[1]|(tvec[1]^ca);
						u[2]=tDir[2]|(ba^tvec[2]); v[2]=tDir[2]|(tvec[2]^ca);
						u[3]=tDir[3]|(ba^tvec[3]); v[3]=tDir[3]|(tvec[3]^ca);
					}
				
					floatq nrmLen=floatq( ((float*)&obj.ca)[3] );

#define COLLIDE(p)  { \
						floatq det=tDir[p]|nrm;		\
						f32x4b mask=Min(u[p],v[p])>=0.0f&&u[p]+v[p]<=det*nrmLen;	\
						if(ForAny(mask)) {		\
							floatq dist=Condition(mask,val[p]/det,out[p]);	\
							mask=dist<out[p]&&dist>0.0f;	\
							out[p]=Condition(mask,Output::type==otShadow?0.00001f:dist,out[p]);	\
							if(Output::objectIndexes)	\
								object[p]=Condition(i32x4b(mask),i32x4(idx),object[p]);	\
							stats.IntersectPass();	\
						} else stats.IntersectFail();	\
					}

					COLLIDE(0)
					COLLIDE(1)
					COLLIDE(2)
					COLLIDE(3)

#undef COLLIDE
				}

			POP_STACK:
				if(fStack==fStackBegin) break;

				fStack-=8;
				tMin[0]=fStack[0];
				tMin[1]=fStack[1];
				tMin[2]=fStack[2];
				tMin[3]=fStack[3];
				tMax[0]=Min(fStack[4],out[0]);
				tMax[1]=Min(fStack[5],out[1]);
				tMax[2]=Min(fStack[6],out[2]);
				tMax[3]=Min(fStack[7],out[3]);
				--nStack;
				idx=*nStack;
				continue;
			}

			const BIHNode *node=node0+(idx&BIHNode::idxMask);
			pattern.Touch(idx&BIHNode::idxMask,4);

			int axis=node->Axis();
			int nidx=dirMask&(1<<axis)?1:0;

			floatq near[4],far[4]; {
				const floatq *inv=tinv[axis];

				if(sharedOrigin) {
					float tnear=node->clip[0]-sharedOrig[axis],tfar=node->clip[1]-sharedOrig[axis];
					if(nidx) Swap(tnear,tfar);
					
					near[0]=Min( floatq(tnear)*inv[0], tMax[0]);
					near[1]=Min( floatq(tnear)*inv[1], tMax[1]);
					near[2]=Min( floatq(tnear)*inv[2], tMax[2]);
					near[3]=Min( floatq(tnear)*inv[3], tMax[3]);

					far [0]=Max( floatq(tfar)*inv[0], tMin[0]);
					far [1]=Max( floatq(tfar)*inv[1], tMin[1]);
					far [2]=Max( floatq(tfar)*inv[2], tMin[2]);
					far [3]=Max( floatq(tfar)*inv[3], tMin[3]);
				}
				else {
					floatq *start=torig[axis];
					float tnear=node->clip[0],tfar=node->clip[1];
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
	}

