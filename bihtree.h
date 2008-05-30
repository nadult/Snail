#ifndef RTRACER_BIHTREE_H
#define RTRACER_BIHTREE_H

#include "object.h"
#include "ray_group.h"
#include "tree_stats.h"
#include "context.h"

typedef TTriangle<SlowEdgeNormals> BIHTriangle;

// Liscie nie sa przechowywane w drzewie
// Zamiast odnosnika do liscia jest od razu odnosnik do obiektu
class BIHNode {
public:
	enum { leafMask=1<<29, idxMask=0x1fffffff };

	inline i32 Child1() const { return val[0]&0x1fffffff; }
	inline i32 Child2() const { return val[1]&0x1fffffff; }
	inline i32 Child(uint idx) const { return val[idx]&0x1fffffff; }

	inline bool Child1IsLeaf() const { return val[0]&leafMask; }
	inline bool Child2IsLeaf() const { return val[1]&leafMask; }
	inline bool ChildIsLeaf(uint idx) const { return val[idx]&leafMask; }

	inline int Axis() const { return (val[0]>>30); }
	inline float ClipLeft() const { return clip[0]; }
	inline float ClipRight() const { return clip[1]; }

	inline bool HasChild1() const { return val[1]&(1<<30); }
	inline bool HasChild2() const { return val[1]&(1<<31); }

	float clip[2];
	u32 val[2];
};

template <class T>
class Vector
{
public:
	typedef T value_type;

	Vector() :tab(0),count(0) { }

	template <class Container>
	Vector(const Container &obj) :tab(0) {
		Alloc(obj.size());
		for(int n=0;n<count;n++) tab[n]=obj[n];
	}
	Vector(const Vector &obj) :tab(0) {
		Alloc(obj.size());
		for(int n=0;n<count;n++) tab[n]=obj[n];
	}
	const Vector &operator=(const Vector &obj) {
		if(&obj==this) return *this;
		return operator=<Vector>(obj);
	}
	template <class Container>
	const Vector &operator=(const Container &obj) {
		Alloc(obj.size());
		for(int n=0;n<count;n++) tab[n]=obj[n];
		return *this;
	}
	~Vector() { Free(); }

	inline size_t size() const { return count; }
	inline const T &operator[](int n) const { return tab[n]; }
	inline T &operator[](int n) { return tab[n]; }

private:
	void Alloc(size_t newS) { Free(); count=newS; tab=count?new T[count]:0; }
	void Free() { if(tab) delete[] tab; } 

	T *tab;
	size_t count;
};

class BIHIdx {
public:
	BIHIdx() { }
	BIHIdx(int i,const Vec3f &mi,const Vec3f &ma,float mul) :idx(i),min(mi),max(ma) {
		Vec3f vsize=max-min;
		size=Max(vsize.x,Max(vsize.y,vsize.z))*mul;
	}

	Vec3f min,max;
	float size;
	i32 idx;
};


template <class TObject=BIHTriangle>
class BIHTree {
public:
	typedef TObject Object;
	enum { maxLevel=60 };

	BIHTree(const vector<Object> &obj);

	void PrintInfo() const;
	uint FindSimilarParent(vector<u32> &parents,uint nNode,uint axis) const;
	void Build(vector<BIHIdx> &indices,vector<u32> &parents,uint nNode,int first,int last,Vec3p min,Vec3p max,uint level);

	void FillDSignArray(int dirMask,int *dSign) const {
		dSign[0]=dirMask&1?1:0;
		dSign[1]=dirMask&2?1:0;
		dSign[2]=dirMask&4?1:0;
	}

	template <class Output>
	void TraverseMono(const Vec3p &rOrigin,const Vec3p &tDir,Output output) const {
		float maxD=output.dist[0];

		TreeStats stats;
		stats.TracingRay();

		Vec3p rDir=Vec3p(tDir.x+0.000000000001f,tDir.y+0.000000000001f,tDir.z+0.000000000001f);
		Vec3p invDir=VInv(rDir);

		int signMask=SignMask(floatq(invDir.m));
		int dSign[3]={signMask&1?1:0,signMask&2?1:0,signMask&4?1:0};
		float minRet=maxD,tMin=ConstEpsilon<float>(),tMax=maxD;

		struct Locals { float tMin,tMax; u32 idx; } stackBegin[maxLevel+2],*stack=stackBegin;
		const BIHNode *node,*node0=&nodes[0];
		int idx=0;

		tMax=Min(tMax,minRet);

		{
			Vec3p ttMin=(pMin-rOrigin)*invDir;
			Vec3p ttMax=(pMax-rOrigin)*invDir;
			if(dSign[0]) Swap(ttMin.x,ttMax.x);
			if(dSign[1]) Swap(ttMin.y,ttMax.y);
			if(dSign[2]) Swap(ttMin.z,ttMax.z);

			tMax=Min(Min(ttMax.x,ttMax.y),tMax);
			tMax=Min(ttMax.z,tMax);
			
			tMin=Max(Max(ttMin.x,ttMin.y),tMin);
			tMin=Max(ttMin.z,tMin);
		}

		while(true) {
			stats.LoopIteration();
			if(tMin>tMax) goto POP_STACK;
		
			if(idx&BIHNode::leafMask) {
				idx&=BIHNode::idxMask;
				{
					stats.Intersection();
					const Object &obj=objects[idx];
					float ret=obj.Collide(rOrigin,tDir);
					if(ret<minRet&&ret>0) {
						minRet=ret;
						if(Output::objectIndexes)
							output.object[0]=idx;

						tMax=Min(tMax,minRet);
					}	
				}
POP_STACK:
				if(stack==stackBegin) break;
				stack--;
				tMin=stack->tMin;
				tMax=Min(stack->tMax,minRet);
				idx=stack->idx;
				continue;
			}

			node=node0+(idx&BIHNode::idxMask);
			int axis=node->Axis();
			int nidx=dSign[axis],fidx=nidx^1;

			float near,far; {
				float start=(&rOrigin.x)[axis],inv=(&invDir.x)[axis];
				near=(node->clip[nidx]-start)*inv;
				far =(node->clip[fidx]-start)*inv;
			}

			if(tMin>near) {
				if(tMax<far) goto POP_STACK;

				tMin=Max(tMin,far);
				idx=node->val[fidx];
				continue;
			}
			if(tMax<far) {
				if(tMin>near) goto POP_STACK;

				tMax=Min(tMax,near);
				idx=node->val[nidx];
				continue;
			}

			stack->tMin=Max(tMin,far);
			stack->tMax=tMax;
			stack->idx=node->val[fidx];
			stack++;

			tMax=Min(tMax,near);
			idx=node->val[nidx];
		}

		output.stats->Update(stats);
		output.dist[0]=minRet;
	}

	template <class Output>
	void TraverseQuad(const Vec3q &rOrigin,const Vec3q &tDir,Output output,int dirMask) const {
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
				if(ForAll(tMin>near)) goto POP_STACK;

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

	template <class Output>
	void TraverseQuad4(const Vec3q *rOrigin,const Vec3q *tDir,floatq *out,i32x4 *object,TreeStats *tstats,int dirMask) const {
		floatq maxD[4]={out[0],out[1],out[2],out[3]};

		TreeStats stats;
		stats.TracingPacket(16);

		Vec3q invDir[4]={
			VInv(Vec3q(tDir[0].x+0.000000000001f,tDir[0].y+0.000000000001f,tDir[0].z+0.000000000001f)),
			VInv(Vec3q(tDir[1].x+0.000000000001f,tDir[1].y+0.000000000001f,tDir[1].z+0.000000000001f)),
			VInv(Vec3q(tDir[2].x+0.000000000001f,tDir[2].y+0.000000000001f,tDir[2].z+0.000000000001f)),
			VInv(Vec3q(tDir[3].x+0.000000000001f,tDir[3].y+0.000000000001f,tDir[3].z+0.000000000001f)) };
		floatq tinv[3][4]={
			{invDir[0].x,invDir[1].x,invDir[2].x,invDir[3].x},
			{invDir[0].y,invDir[1].y,invDir[2].y,invDir[3].y},
			{invDir[0].z,invDir[1].z,invDir[2].z,invDir[3].z} };
		floatq torig[3][4]={
			{rOrigin[0].x,rOrigin[1].x,rOrigin[2].x,rOrigin[3].x},
			{rOrigin[0].y,rOrigin[1].y,rOrigin[2].y,rOrigin[3].y},
			{rOrigin[0].z,rOrigin[1].z,rOrigin[2].z,rOrigin[3].z} };

		int dSign[3]; FillDSignArray(dirMask,dSign);
		floatq minRet[4]={maxD[0],maxD[1],maxD[2],maxD[3]},tMin[4],tMax[4];
		tMin[0]=tMin[1]=tMin[2]=tMin[3]=ConstEpsilon<floatq>();
		tMax[0]=Min(maxD[0],minRet[0]);
		tMax[1]=Min(maxD[1],minRet[1]);
		tMax[2]=Min(maxD[2],minRet[2]);
		tMax[3]=Min(maxD[3],minRet[3]);

		floatq fStackBegin[8*(maxLevel+2)],*fStack=fStackBegin;
		u32 nStackBegin[maxLevel+2],*nStack=nStackBegin;

		{
			Vec3q ttMin[4]={
				(Vec3q(pMin)-rOrigin[0])*invDir[0],
				(Vec3q(pMin)-rOrigin[1])*invDir[1],
				(Vec3q(pMin)-rOrigin[2])*invDir[2],
				(Vec3q(pMin)-rOrigin[3])*invDir[3] };
			Vec3q ttMax[4]={
				(Vec3q(pMax)-rOrigin[0])*invDir[0],
				(Vec3q(pMax)-rOrigin[1])*invDir[1],
				(Vec3q(pMax)-rOrigin[2])*invDir[2],
				(Vec3q(pMax)-rOrigin[3])*invDir[3] };

			if(dSign[0]) {
				Swap(ttMin[0].x,ttMax[0].x);
				Swap(ttMin[1].x,ttMax[1].x);
				Swap(ttMin[2].x,ttMax[2].x);
				Swap(ttMin[3].x,ttMax[3].x); }
			if(dSign[1]) {
				Swap(ttMin[0].y,ttMax[0].y);
				Swap(ttMin[1].y,ttMax[1].y);
				Swap(ttMin[2].y,ttMax[2].y);
				Swap(ttMin[3].y,ttMax[3].y); }
			if(dSign[2]) {
				Swap(ttMin[0].z,ttMax[0].z);
				Swap(ttMin[1].z,ttMax[1].z);
				Swap(ttMin[2].z,ttMax[2].z);
				Swap(ttMin[3].z,ttMax[3].z); }

			tMax[0]=Min(Min(ttMax[0].x,ttMax[0].y),tMax[0]);
			tMax[1]=Min(Min(ttMax[1].x,ttMax[1].y),tMax[1]);
			tMax[2]=Min(Min(ttMax[2].x,ttMax[2].y),tMax[2]);
			tMax[3]=Min(Min(ttMax[3].x,ttMax[3].y),tMax[3]);

			tMax[0]=Min(ttMax[0].z,tMax[0]);
			tMax[1]=Min(ttMax[1].z,tMax[1]);
			tMax[2]=Min(ttMax[2].z,tMax[2]);
			tMax[3]=Min(ttMax[3].z,tMax[3]);
			
			tMin[0]=Max(Max(ttMin[0].x,ttMin[0].y),tMin[0]);
			tMin[1]=Max(Max(ttMin[1].x,ttMin[1].y),tMin[1]);
			tMin[2]=Max(Max(ttMin[2].x,ttMin[2].y),tMin[2]);
			tMin[3]=Max(Max(ttMin[3].x,ttMin[3].y),tMin[3]);

			tMin[0]=Max(ttMin[0].z,tMin[0]);
			tMin[1]=Max(ttMin[1].z,tMin[1]);
			tMin[2]=Max(ttMin[2].z,tMin[2]);
			tMin[3]=Max(ttMin[3].z,tMin[3]);
		}
		ObjectIdxBuffer<4> mailbox;

		const BIHNode *node0=&nodes[0];
		int idx=0;

		int done[4]={0,0,0,0};

		while(true) {
			stats.LoopIteration();

			if(idx&BIHNode::leafMask) {
				idx&=BIHNode::idxMask;

				if(!mailbox.Find(idx)) {
					mailbox.Insert(idx);
					stats.Intersection(4);

					const Object &obj=objects[idx];

					Vec3q tvec[4]; {
						Vec3q a(obj.a.x,obj.a.y,obj.a.z);
						tvec[0]=rOrigin[0]-a;
						tvec[1]=rOrigin[1]-a;
						tvec[2]=rOrigin[2]-a;
						tvec[3]=rOrigin[3]-a;
					}
					floatq u[4],v[4]; {
						Vec3q ba(obj.ba.x,obj.ba.y,obj.ba.z),ca(obj.ca.x,obj.ca.y,obj.ca.z);
						u[0]=tDir[0]|(ba^tvec[0]);
						v[0]=tDir[0]|(tvec[0]^ca);
						u[1]=tDir[1]|(ba^tvec[1]);
						v[1]=tDir[1]|(tvec[1]^ca);
						u[2]=tDir[2]|(ba^tvec[2]);
						v[2]=tDir[2]|(tvec[2]^ca);
						u[3]=tDir[3]|(ba^tvec[3]);
						v[3]=tDir[3]|(tvec[3]^ca);
					}

					Vec3p nrm=obj.Nrm();
					floatq nrmLen=floatq( ((float*)&obj.ca)[3] );
					{
						floatq det=tDir[0]|nrm;
						f32x4b mask=Min(u[0],v[0])>=0.0f&&u[0]+v[0]<=det*nrmLen;
						if(ForAny(mask)) {
							floatq dist=Condition(mask,-(tvec[0]|nrm)/det,minRet[0]);
							mask=dist<minRet[0]&&dist>0.0f;
							if(Output::type==otShadow) done[0]|=ForWhich(mask);
							minRet[0]=Condition(mask,dist,minRet[0]);
							if(Output::objectIndexes)
								object[0]=Condition(i32x4b(mask),i32x4(idx),object[0]);
						}
					} {
						floatq det=tDir[1]|nrm;
						f32x4b mask=Min(u[1],v[1])>=0.0f&&u[1]+v[1]<=det*nrmLen;
						if(ForAny(mask)) {
							floatq dist=Condition(mask,-(tvec[1]|nrm)/det,minRet[1]);
							mask=dist<minRet[1]&&dist>0.0f;
							if(Output::type==otShadow) done[1]|=ForWhich(mask);
							minRet[1]=Condition(mask,dist,minRet[1]);
							if(Output::objectIndexes)
								object[1]=Condition(i32x4b(mask),i32x4(idx),object[1]);
						}
					} {
						floatq det=tDir[2]|nrm;
						f32x4b mask=Min(u[2],v[2])>=0.0f&&u[2]+v[2]<=det*nrmLen;
						if(ForAny(mask)) {
							floatq dist=Condition(mask,-(tvec[2]|nrm)/det,minRet[2]);
							mask=dist<minRet[2]&&dist>0.0f;
							if(Output::type==otShadow) done[2]|=ForWhich(mask);
							minRet[2]=Condition(mask,dist,minRet[2]);
							if(Output::objectIndexes)
								object[2]=Condition(i32x4b(mask),i32x4(idx),object[2]);
						}
					} {
						floatq det=tDir[3]|nrm;
						f32x4b mask=Min(u[3],v[3])>=0.0f&&u[3]+v[3]<=det*nrmLen;
						if(ForAny(mask)) {
							floatq dist=Condition(mask,-(tvec[3]|nrm)/det,minRet[3]);
							mask=dist<minRet[3]&&dist>0.0f;
							if(Output::type==otShadow) done[3]|=ForWhich(mask);
							minRet[3]=Condition(mask,dist,minRet[3]);
							if(Output::objectIndexes)
								object[3]=Condition(i32x4b(mask),i32x4(idx),object[3]);
						}
					}

					if(Output::type==otShadow)
						if((done[0]&done[1]&done[2]&done[3])==15)
							break;
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
			int nidx=dSign[axis];
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
				if(ForAll(test1)) goto POP_STACK;

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

	template <class Output,class Group>
	void TraverseMonoGroup(Group &group,const RaySelector<Group::size> &sel,const Output &out) const {
		Vec3p orig[4],dir[4];
		u32 tmp[4];

		if(Group::sharedOrigin)
			Convert(group.Origin(sel[0]),orig);
		
		for(int i=0;i<sel.Num();i++) {
			int q=sel[i];
			Vec3p dir[4];
			int bmask=sel.BitMask(i);

			if(!Group::sharedOrigin)
				Convert(group.Origin(q),orig);
			Convert(group.Dir(q),dir);

			float *dist=(float*)(out.dist+q);
			u32 *objId=Output::objectIndexes?(u32*)(out.object+q):tmp;

			if(bmask&1) TraverseMono(orig[0],dir[0],::Output<otNormal,float,u32>(dist+0,objId+0,out.stats));
			if(bmask&2) TraverseMono(orig[1],dir[1],::Output<otNormal,float,u32>(dist+1,objId+1,out.stats));
			if(bmask&4) TraverseMono(orig[2],dir[2],::Output<otNormal,float,u32>(dist+2,objId+2,out.stats));
			if(bmask&8) TraverseMono(orig[3],dir[3],::Output<otNormal,float,u32>(dist+3,objId+3,out.stats));
		}
	}

	template <class Output,class Rays>
	void TraverseQuadGroup(Rays &rays,const RaySelector<Rays::size> &sel,const Output &out,int dirMask) const {
		int i=0;
		for(;i+3<sel.Num();i+=4) {
			int q[4]={sel[i],sel[i+1],sel[i+2],sel[i+3]};
			Vec3q tOrig[4],tDir[4];
			floatq dist[4]; i32x4 obj[4];

			dist[0]=out.dist[q[0]]; dist[1]=out.dist[q[1]];
			dist[2]=out.dist[q[2]]; dist[3]=out.dist[q[3]];
			if(Output::objectIndexes) {
				obj[0]=out.object[q[0]]; obj[1]=out.object[q[1]];
				obj[2]=out.object[q[2]]; obj[3]=out.object[q[3]];
			}
			tOrig[0]=rays.Origin(q[0]); tOrig[1]=rays.Origin(q[1]);
			tOrig[2]=rays.Origin(q[2]); tOrig[3]=rays.Origin(q[3]);
			tDir[0]=rays.Dir(q[0]); tDir[1]=rays.Dir(q[1]);
			tDir[2]=rays.Dir(q[2]); tDir[3]=rays.Dir(q[3]);

			TraverseQuad4<Output>(tOrig,tDir,dist,obj,out.stats,dirMask);

			out.dist[q[0]]=dist[0]; out.dist[q[1]]=dist[1];
			out.dist[q[2]]=dist[2]; out.dist[q[3]]=dist[3];
			if(Output::objectIndexes) {
				out.object[q[0]]=obj[0]; out.object[q[1]]=obj[1];
				out.object[q[2]]=obj[2]; out.object[q[3]]=obj[3];
			}
		}
		for(;i<sel.Num();i++) {
			int q=sel[i];
			TraverseQuad(rays.Origin(q),rays.Dir(q),Output(out.dist+q,out.object+q,out.stats),dirMask);
		}
	}


	template <class Output,class Rays>
	void TraverseOptimized(Rays &rays,const RaySelector<Rays::size> &sel,const Output &out) const {
		if(!sel.Num()) return;

		RaySelector<Rays::size> selectors[9];
		rays.GenSelectors(sel,selectors);

		for(int k=0;k<8;k++) {
			RaySelector<Rays::size> &sel=selectors[k];
			if(sel.Num()) {
				// Ulepszyc, ta czworka moze byc popsuta (wektorki w roznych kierunkach)
				Vec3q dir0=rays.Dir(sel[0]);

				if(Output::type!=otPrimary) for(int i=1;i<sel.Num();i++) {
					int q=sel[i];

					int bitMask=sel.BitMask(i);
					if(bitMask!=15) { selectors[8].Add(q,bitMask); sel.Disable(i--); continue; }
	
					floatq dot=rays.Dir(q)|dir0;
					if(ForAny(dot<Const<floatq,998,1000>())) { selectors[8].Add(q); sel.Disable(i--); continue; }
				}
				TraverseQuadGroup(rays,sel,out,k);
			}
		}
		if(selectors[8].Num())
			TraverseMonoGroup(rays,selectors[8],out);
	}

	int vLeafs;
	Vec3p pMin,pMax;
	vector<BIHNode> nodes;
	Vector<BIHTriangle> objects;
};

#include "bihtree.inl"

#endif

