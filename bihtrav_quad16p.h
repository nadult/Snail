
	// Its fast when hitting big triangles (relatively)
	// When you have a lot of small triangles, TraverseQuad4 will be faster
	template <class Output>
	void BIHTree::TraverseQuad16Primary(const BIHTravContext &c,int dirMask) const {
		TreeStats stats;
		stats.TracingPacket(64);

		Vec3p orig=Vec3p(c.origin[0].x[0],c.origin[0].y[0],c.origin[0].z[0]);

		Vec3p tMinInv[4],tMaxInv[4];
		Vec3f maxDir; {
			Vec3q t=c.dir[0]; for(int k=1;k<16;k++) t=VMax(t,c.dir[k]);
			maxDir=Maximize(t);
		}
		Vec3p maxInv,minInv; {
			for(int k=0;k<4;k++) {
				Vec3p tMax,tMin; {
					Vec3q aDir[4]={ VAbs(c.dir[k*4+0]),VAbs(c.dir[k*4+1]),VAbs(c.dir[k*4+2]),VAbs(c.dir[k*4+3]) };
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
		Vec3p beamDir; float beamM; {
			beamDir=Vec3p(0,0,0);
			Vec3q maxVec=c.dir[0];
			Vec3q tmp=maxVec;
			for(int n=1;n<16;n++) tmp+=c.dir[n];

			Vec3p t[4]; Convert(tmp,t);
			beamDir=t[0]+t[1]+t[2]+t[3];
			beamDir*=RSqrt(beamDir|beamDir);

			floatq minDot=Const<floatq,1>();
			Vec3q mid=Vec3q(beamDir);

			for(int n=0;n<16;n++) {
				floatq dot=c.dir[n]|mid;

				f32x4b mask=dot<minDot;
				maxVec=Condition(mask,c.dir[n],maxVec);
				minDot=Condition(mask,dot,minDot);
			}

			floatq maxDist=Length(maxVec-mid*minDot);
			beamM=Maximize(maxDist);
		};

		float tMaxMax=Maximize(c.out[0]);

		float fStackBegin[2*(maxLevel+2)],* __restrict__ fStack=fStackBegin;
		u32 nStackBegin[maxLevel+2],* __restrict__ nStack=nStackBegin;

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

		const  BIHNode *node0=&nodes[0];
		int idx=0;

		float allTMin=tMin,allTMax=tMax;
		int allIdx=idx;
		float lastDensity=0;
//
		while(true) {
			stats.LoopIteration();
			if(idx&BIHNode::leafMask) {
				idx&=BIHNode::idxMask;

				if(!mailbox.Find(idx)) {

					const BIHTriangle &obj=objects[idx];
					Vec3p nrm=obj.Nrm();
					const floatq nrmLen=floatq( ((float*)&obj.ca)[3] );

					{
						float dot=beamDir|nrm;
						float t=((Vec3f(orig)-obj.a)|nrm);
						if(t>0.0f||dot<0.0f) goto POP_STACK;

						float idot=Inv(dot);
						t=-t*idot;

						Vec3f col=orig+beamDir*t;
						float epsilon=beamM*t*idot;

						float distA=(col-obj.P1())|obj.Edge1Normal();
						float distB=(col-obj.P2())|obj.Edge2Normal();
						float distC=(col-obj.P3())|obj.Edge3Normal();
						float min=Min(distA,Min(distB,distC));

						if(min<-epsilon) { mailbox.Insert(idx); goto POP_STACK; }
						if(min>epsilon) {
							mailbox.Insert(idx);
							stats.IntersectPass(16);
							stats.Intersection(16);
							floatq val=-((orig-obj.a)|nrm);

							#define COLLIDE(p)	{ \
								floatq det=c.dir[p]|nrm; \
								floatq dist=val/det; \
								f32x4b mask=dist<c.out[p]; \
								c.out[p]=Condition(mask,Output::type==otShadow?0.00001f:dist,c.out[p]); \
								if(Output::objectIndexes) { \
									c.element[p]=Condition(i32x4b(mask),i32x4(idx),c.element[p]); \
									c.object[p]=Condition(i32x4b(mask),i32x4(objectId),c.object[p]); \
								} \
							}
							
							floatq ttMax[4];
							COLLIDE(0) COLLIDE(1) COLLIDE(2) COLLIDE(3)
							ttMax[0]=Max(Max(c.out[0],c.out[1]),Max(c.out[2],c.out[3]));
							COLLIDE(4) COLLIDE(5) COLLIDE(6) COLLIDE(7)
							ttMax[1]=Max(Max(c.out[4],c.out[5]),Max(c.out[6],c.out[7]));
							COLLIDE(8) COLLIDE(9) COLLIDE(10) COLLIDE(11)
							ttMax[2]=Max(Max(c.out[8],c.out[9]),Max(c.out[10],c.out[11]));
							COLLIDE(12) COLLIDE(13) COLLIDE(14) COLLIDE(15)
							ttMax[3]=Max(Max(c.out[12],c.out[13]),Max(c.out[14],c.out[15]));
							tMaxMax=Maximize(Max(Max(ttMax[0],ttMax[1]),Max(ttMax[2],ttMax[3])));
							#undef COLLIDE

							goto POP_STACK;
						}
					}	
			
			/*		if(lastDensity*(tMin+tMax)>maxDensity) {
					//	stats.Skip(16);
						if(fStack==fStackBegin) { allTMin=tMin; allTMax=tMax; allIdx=idx|BIHNode::leafMask; }

						BIHOptData data(orig,tMinInv+0,tMaxInv+0,mailbox,allTMin,allTMax,allIdx);
						BIHTravContext cn(c.origin,c.dir,c.out,c.object,c.element,c.stats);
						TraverseQuad4Primary<Output>(cn,dirMask,&data);

						cn.origin+=4; cn.dir+=4; cn.out+=4; cn.object+=4; data.minInv++; data.maxInv++;
						TraverseQuad4Primary<Output>(cn,dirMask,&data);

						cn.origin+=4; cn.dir+=4; cn.out+=4; cn.object+=4; data.minInv++; data.maxInv++;
						TraverseQuad4Primary<Output>(cn,dirMask,&data);

						cn.origin+=4; cn.dir+=4; cn.out+=4; cn.object+=4; data.minInv++; data.maxInv++;
						TraverseQuad4Primary<Output>(cn,dirMask,&data);

						break;
					} */

					mailbox.Insert(idx);

					floatq u[16],v[16],val; {
						Vec3f tvec=orig-obj.a;
						Vec3f ba(obj.ba.x,obj.ba.y,obj.ba.z),ca(obj.ca.x,obj.ca.y,obj.ca.z);
						Vec3f cross1=ba^tvec,cross2=tvec^ca;

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


					#define COLLIDE(p)	{ \
						floatq det=c.dir[p]|nrm; \
						f32x4b mask=Min(u[p],v[p])>=0.0f&&u[p]+v[p]<=det*nrmLen; \
						stats.Intersection(); \
						if(ForAny(mask)) { \
							floatq dist=Condition(mask,val/det,c.out[p]); \
							mask=dist<c.out[p]&&dist>0.0f; \
							c.out[p]=Condition(mask,Output::type==otShadow?0.00001f:dist,c.out[p]); \
							if(Output::objectIndexes) { \
								c.element[p]=Condition(i32x4b(mask),i32x4(idx),c.element[p]); \
								c.object[p]=Condition(i32x4b(mask),i32x4(objectId),c.object[p]); \
							} \
							stats.IntersectPass(); \
						} else stats.IntersectFail(); \
					}
					
					floatq ttMax[4];
					COLLIDE(0) COLLIDE(1) COLLIDE(2) COLLIDE(3)
					ttMax[0]=Max(Max(c.out[0],c.out[1]),Max(c.out[2],c.out[3]));
					COLLIDE(4) COLLIDE(5) COLLIDE(6) COLLIDE(7)
					ttMax[1]=Max(Max(c.out[4],c.out[5]),Max(c.out[6],c.out[7]));
					COLLIDE(8) COLLIDE(9) COLLIDE(10) COLLIDE(11)
					ttMax[2]=Max(Max(c.out[8],c.out[9]),Max(c.out[10],c.out[11]));
					COLLIDE(12) COLLIDE(13) COLLIDE(14) COLLIDE(15)
					ttMax[3]=Max(Max(c.out[12],c.out[13]),Max(c.out[14],c.out[15]));
					tMaxMax=Maximize(Max(Max(ttMax[0],ttMax[1]),Max(ttMax[2],ttMax[3])));
					#undef COLLIDE
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

			idx&=BIHNode::idxMask;
			const BIHNode *node=node0+idx;
			pattern.Touch(idx&BIHNode::idxMask,16);
	//		lastDensity=node->density;

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

