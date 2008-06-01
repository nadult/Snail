

	// Its fast when hitting big triangles (relatively)
	// When you have a lot of small triangles, TraverseQuad4 will be faster
	template <class AccStruct> template <class Output>
	void BIHTree<AccStruct>:: TraverseQuad16Primary(const BIHTravContext &c,int dirMask) const {
		TreeStats stats;
		stats.TracingPacket(64);

		Vec3p orig=Vec3p(c.origin[0].x[0],c.origin[0].y[0],c.origin[0].z[0]);

		Vec3p tMinInv[4],tMaxInv[4];
		Vec3p maxInv,minInv; {
			for(int k=0;k<4;k++) {
				Vec3p tMax,tMin; {
					Vec3q aDir[4]={VAbs(c.dir[k*4+0]),VAbs(c.dir[k*4+1]),VAbs(c.dir[k*4+2]),VAbs(c.dir[k*4+3])};
					tMax=Maximize( VMax(VMax(aDir[0],aDir[1]),VMax(aDir[2],aDir[3])) );
					tMin=Minimize( VMin(VMin(aDir[0],aDir[1]),VMin(aDir[2],aDir[3])) );
				}
				Vec3p shift(0.000000000001f,0.000000000001f,0.000000000001f);
				(&tMin.x)[3]=(&tMax.x)[3]=1.0f;
				tMin=VInv(tMin+shift);
				tMax=VInv(tMax+shift);
				if(dirMask&1) { tMin.x=-tMin.x; tMax.x=-tMax.x; }
				if(dirMask&2) { tMin.y=-tMin.y; tMax.y=-tMax.y; }
				if(dirMask&4) { tMin.z=-tMin.z; tMax.z=-tMax.z; }
				tMinInv[k]=VMin(tMin,tMax);
				tMaxInv[k]=VMax(tMin,tMax);
			}
			minInv=VMin(VMin(tMinInv[0],tMinInv[1]),VMin(tMinInv[2],tMinInv[3]));
			maxInv=VMax(VMax(tMaxInv[0],tMaxInv[1]),VMax(tMaxInv[2],tMaxInv[3]));
		}

		float tMaxMax=Maximize(c.out[0]);

		float fStackBegin[2*(maxLevel+2)],*fStack=fStackBegin;
		u32 nStackBegin[maxLevel+2],*nStack=nStackBegin;

		float tMax,tMin; {
			Vec3p rMin=pMin,rMax=pMax;
			if(dirMask&1) Swap(rMin.x,rMax.x);
			if(dirMask&2) Swap(rMin.y,rMax.y);
			if(dirMask&4) Swap(rMin.z,rMax.z);

			Vec3p ttMin=VMin((rMin-orig)*minInv,(rMin-orig)*maxInv);
			Vec3p ttMax=VMax((rMax-orig)*minInv,(rMax-orig)*maxInv);

			tMin=Max(Max(0.000001f,ttMin.x),Max(ttMin.y,ttMin.z));
			tMax=Min(Min(tMaxMax  ,ttMax.x),Min(ttMax.y,ttMax.z));
		}
		ObjectIdxBuffer<4> mailbox;

		const BIHNode *node0=&nodes[0];
		int idx=0,full=0;

		float allTMin=tMin,allTMax=tMax;
		int allIdx=idx;

		while(true) {
			stats.LoopIteration();
			if(idx&BIHNode::leafMask) {
				idx&=BIHNode::idxMask;

				if(!mailbox.Find(idx)) {
					full++;
					mailbox.Insert(idx);
					stats.Intersection(16);

					const Object &obj=objects[idx];

					Vec3p nrm=obj.Nrm();
					floatq u[16],v[16],val; {
						Vec3q tvec=c.origin[0]-Vec3q(obj.a);
						Vec3q ba(obj.ba.x,obj.ba.y,obj.ba.z),ca(obj.ca.x,obj.ca.y,obj.ca.z);
						Vec3q cross1=ba^tvec,cross2=tvec^ca;

						u[ 0]=c.dir[ 0]|cross1; v[ 0]=c.dir[ 0]|cross2;
						u[ 1]=c.dir[ 1]|cross1; v[ 1]=c.dir[ 1]|cross2;
						u[ 2]=c.dir[ 2]|cross1; v[ 2]=c.dir[ 2]|cross2;
						u[ 3]=c.dir[ 3]|cross1; v[ 3]=c.dir[ 3]|cross2;
						u[ 4]=c.dir[ 4]|cross1; v[ 4]=c.dir[ 4]|cross2;
						u[ 5]=c.dir[ 5]|cross1; v[ 5]=c.dir[ 5]|cross2;
						u[ 6]=c.dir[ 6]|cross1; v[ 6]=c.dir[ 6]|cross2;
						u[ 7]=c.dir[ 7]|cross1; v[ 7]=c.dir[ 7]|cross2;
						u[ 8]=c.dir[ 8]|cross1; v[ 8]=c.dir[ 8]|cross2;
						u[ 9]=c.dir[ 9]|cross1; v[ 9]=c.dir[ 9]|cross2;
						u[10]=c.dir[10]|cross1; v[10]=c.dir[10]|cross2;
						u[11]=c.dir[11]|cross1; v[11]=c.dir[11]|cross2;
						u[12]=c.dir[12]|cross1; v[12]=c.dir[12]|cross2;
						u[13]=c.dir[13]|cross1; v[13]=c.dir[13]|cross2;
						u[14]=c.dir[14]|cross1; v[14]=c.dir[14]|cross2;
						u[15]=c.dir[15]|cross1; v[15]=c.dir[15]|cross2;

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
					
					floatq ttMax[4];
					COLLIDE(0)
					COLLIDE(1)
					COLLIDE(2)
					COLLIDE(3)
					ttMax[0]=Max(Max(c.out[0],c.out[1]),Max(c.out[2],c.out[3]));
					COLLIDE(4)
					COLLIDE(5)
					COLLIDE(6)
					COLLIDE(7)
					ttMax[1]=Max(Max(c.out[4],c.out[5]),Max(c.out[6],c.out[7]));
					COLLIDE(8)
					COLLIDE(9)
					COLLIDE(10)
					COLLIDE(11)
					ttMax[2]=Max(Max(c.out[8],c.out[9]),Max(c.out[10],c.out[11]));
					COLLIDE(12)
					COLLIDE(13)
					COLLIDE(14)
					COLLIDE(15)
					ttMax[3]=Max(Max(c.out[12],c.out[13]),Max(c.out[14],c.out[15]));
					#undef COLLIDE

					tMaxMax=Maximize(Max(Max(ttMax[0],ttMax[1]),Max(ttMax[2],ttMax[3])));
				}

			POP_STACK:
				if(fStack==fStackBegin) break;

				fStack-=2;
				tMin=fStack[0];
				tMax=Min(fStack[1],tMaxMax);

				--nStack;
				idx=*nStack;

				if(fStack==fStackBegin) { allTMin=tMin; allTMax=tMax; allIdx=idx; }
				continue;
			}

			const BIHNode *node=node0+(idx&BIHNode::idxMask);
			pattern.Touch(idx&BIHNode::idxMask,16);

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

			if(/*split&&*/node->density*(tMin+tMax)*0.5f>maxDensity) {
				if(fStack==fStackBegin) { allTMin=tMin; allTMax=tMax; allIdx=idx; }
				stats.Skip();

				BIHOptData data(orig,tMinInv+0,tMaxInv+0,mailbox,allTMin,allTMax,allIdx);
				BIHTravContext cn(c.origin,c.dir,c.out,c.object,c.stats);
				TraverseQuad4Primary<Output>(cn,dirMask,&data);

				cn.origin+=4; cn.dir+=4; cn.out+=4; cn.object+=4; data.minInv++; data.maxInv++;
				TraverseQuad4Primary<Output>(cn,dirMask,&data);

				cn.origin+=4; cn.dir+=4; cn.out+=4; cn.object+=4; data.minInv++; data.maxInv++;
				TraverseQuad4Primary<Output>(cn,dirMask,&data);

				cn.origin+=4; cn.dir+=4; cn.out+=4; cn.object+=4; data.minInv++; data.maxInv++;
				TraverseQuad4Primary<Output>(cn,dirMask,&data);

				break;
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

