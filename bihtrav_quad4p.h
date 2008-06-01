

	// Its fast when hitting big triangles (relatively)
	// When you have a lot of small triangles, TraverseQuad4 will be faster
	template <class AccStruct> template <class Output>
	void BIHTree<AccStruct>:: TraverseQuad4Primary(const BIHTravContext &c,int dirMask,BIHOptData *data) const {
		TreeStats stats;
		stats.TracingPacket(data?0:16);

		const BIHNode *node0=&nodes[0];
		float fStackBegin[2*(maxLevel+2)],*fStack=fStackBegin;
		u32 nStackBegin[maxLevel+2],*nStack=nStackBegin;
		ObjectIdxBuffer<4> mailbox;

		Vec3p orig,maxInv,minInv;
		float tMin,tMax,tMaxMax;
		int idx;

		tMaxMax=Maximize(Max(Max(c.out[0],c.out[1]),Max(c.out[2],c.out[3])));
		if(data) {
			orig=data->orig;
			minInv=*data->minInv;
			maxInv=*data->maxInv;
			tMin=data->min; tMax=Min(data->max,tMaxMax);
			mailbox=data->mailbox;
			idx=data->idx;
		}
		else {
			orig=Vec3p(c.origin[0].x[0],c.origin[0].y[0],c.origin[0].z[0]);
			{
				Vec3q aDir[4]={VAbs(c.dir[0]),VAbs(c.dir[1]),VAbs(c.dir[2]),VAbs(c.dir[3])};
				Vec3p tMax=Maximize( VMax(VMax(aDir[0],aDir[1]),VMax(aDir[2],aDir[3])) );
				Vec3p tMin=Minimize( VMin(VMin(aDir[0],aDir[1]),VMin(aDir[2],aDir[3])) );
				Vec3p shift(0.000000000001f,0.000000000001f,0.000000000001f);
				(&tMin.x)[3]=(&tMax.x)[3]=1.0f;
				tMin=VInv(tMin+shift);
				tMax=VInv(tMax+shift);
				if(dirMask&1) { tMin.x=-tMin.x; tMax.x=-tMax.x; }
				if(dirMask&2) { tMin.y=-tMin.y; tMax.y=-tMax.y; }
				if(dirMask&4) { tMin.z=-tMin.z; tMax.z=-tMax.z; }
				maxInv=VMax(tMin,tMax);
				minInv=VMin(tMin,tMax);
			}
			{
				Vec3p rMin=pMin,rMax=pMax;
				if(dirMask&1) Swap(rMin.x,rMax.x);
				if(dirMask&2) Swap(rMin.y,rMax.y);
				if(dirMask&4) Swap(rMin.z,rMax.z);

				Vec3p ttMin=VMin((rMin-orig)*minInv,(rMin-orig)*maxInv);
				Vec3p ttMax=VMax((rMax-orig)*minInv,(rMax-orig)*maxInv);

				tMin=Max(Max(0.000001f	,ttMin.x),Max(ttMin.y,ttMin.z));
				tMax=Min(Min(tMaxMax	,ttMax.x),Min(ttMax.y,ttMax.z));
			}
			idx=0;
		}


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
						Vec3q tvec=c.origin[0]-Vec3q(obj.a);
						Vec3q ba(obj.ba.x,obj.ba.y,obj.ba.z),ca(obj.ca.x,obj.ca.y,obj.ca.z);
						Vec3q cross1=ba^tvec,cross2=tvec^ca;

						u[0]=c.dir[0]|cross1; v[0]=c.dir[0]|cross2;
						u[1]=c.dir[1]|cross1; v[1]=c.dir[1]|cross2;
						u[2]=c.dir[2]|cross1; v[2]=c.dir[2]|cross2;
						u[3]=c.dir[3]|cross1; v[3]=c.dir[3]|cross2;

						val=-(tvec|nrm);
					}

					floatq nrmLen=floatq( ((float*)&obj.ca)[3] );
					#define COLLIDE(p)	{ \
						floatq det=c.dir[p]|nrm; \
						f32x4b mask=Min(u[p],v[p])>=0.0f&&u[p]+v[p]<=det*nrmLen; \
						if(ForAny(mask)) { \
							floatq dist=Condition(mask,val/det,c.out[p]); \
							mask=dist<c.out[p]&&dist>0.0f; \
							c.out[p]=Condition(mask,Output::type==otShadow?0.00001f:dist,c.out[p]); \
							if(Output::objectIndexes) \
								c.object[p]=Condition(i32x4b(mask),i32x4(idx),c.object[p]); \
							stats.IntersectPass(); \
						} else stats.IntersectFail(); \
					}
					
					COLLIDE(0)
					COLLIDE(1)
					COLLIDE(2)
					COLLIDE(3)
					#undef COLLIDE

					tMaxMax=Maximize( Max(Max(c.out[0],c.out[1]),Max(c.out[2],c.out[3])) );
				}

			POP_STACK:
				if(fStack==fStackBegin) break;

				fStack-=2;
				tMin=fStack[0];
				tMax=Min(fStack[1],tMaxMax);

				--nStack;
				idx=*nStack;
				continue;
			}

			const BIHNode *node=node0+(idx&BIHNode::idxMask);
			pattern.Touch(idx&BIHNode::idxMask,4);

			int axis=node->Axis();
			int nidx=dirMask&(1<<axis)?1:0;

			float near,far; {
				float tNear=node->clip[0]-(&orig.x)[axis];
				float tFar=node->clip[1]-(&orig.x)[axis];
				if(nidx) Swap(tNear,tFar);

				near=Max((&minInv.x)[axis]*tNear,(&maxInv.x)[axis]*tNear);
				near=Min(near,tMax);

				far =Min((&minInv.x)[axis]*tFar	,(&maxInv.x)[axis]*tFar);
				far =Max(far,tMin);
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
			fStack+=2;

			*nStack=node->val[nidx^1];
			nStack++;

			tMax=near;
			idx=node->val[nidx];
		}

		if(c.stats) c.stats->Update(stats);
	}

