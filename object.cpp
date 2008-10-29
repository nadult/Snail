#include "object.h"
#include "bihtree.h"

namespace {

	template <class real,class Vec>
	bool IntersectOBB(const Vec &rDir,const Vec &rOrig,const Vec3f bAxis[3],const float bExtent[3],const Vec3f &bCenter) {
		real fWdU[3],fAWdU[3],fAWxDdU[3],fRhs[3];

		Vec kDiff=rOrig-Vec(bCenter);
		
		fWdU[0] = rDir|bAxis[0];
		fWdU[1] = rDir|bAxis[1];
		fWdU[2] = rDir|bAxis[2];

		{
			real fDdU[3],fADdU[3];
			fDdU[0] = kDiff|bAxis[0];
			fDdU[1] = kDiff|bAxis[1];
			fDdU[2] = kDiff|bAxis[2];

			fADdU[0] = Abs(fDdU[0]);
			fADdU[1] = Abs(fDdU[1]);
			fADdU[2] = Abs(fDdU[2]);

			if(	ForAll(	(fADdU[0]>bExtent[0]&&fDdU[0]*fWdU[0]>=0.0f)||
						(fADdU[1]>bExtent[1]&&fDdU[1]*fWdU[1]>=0.0f)||
						(fADdU[2]>bExtent[2]&&fDdU[2]*fWdU[2]>=0.0f) ))
				return 0;
		}

		Vec kWxD=rDir^kDiff;

		fAWxDdU[0]=Abs(kWxD|bAxis[0]);
		fAWxDdU[1]=Abs(kWxD|bAxis[1]);
		fAWxDdU[2]=Abs(kWxD|bAxis[2]);

		fAWdU[0] = Abs(fWdU[0]);
		fAWdU[1] = Abs(fWdU[1]);
		fAWdU[2] = Abs(fWdU[2]);
		
		fRhs[0]=bExtent[1]*fAWdU[2]+bExtent[2]*fAWdU[1];
		fRhs[1]=bExtent[0]*fAWdU[2]+bExtent[2]*fAWdU[0];
		fRhs[2]=bExtent[0]*fAWdU[1]+bExtent[1]*fAWdU[0];

		return ForAny(fAWxDdU[0]<=fRhs[0]&&fAWxDdU[1]<=fRhs[1]&&fAWxDdU[2]<=fRhs[2]);
	}

	template <class real,class Vec>
	bool IntersectAABB(const Vec &rDir,const Vec &rOrig,const float bExtent[3],const Vec3f &bCenter) {
		Vec kDiff=rOrig-Vec(bCenter);

		if(	ForAll(	(Abs(kDiff.x)>bExtent[0]&&kDiff.x*rDir.x>=0.0f)||
					(Abs(kDiff.y)>bExtent[1]&&kDiff.y*rDir.y>=0.0f)||
					(Abs(kDiff.z)>bExtent[2]&&kDiff.z*rDir.z>=0.0f) ))
			return 0;

		real fAWdU[3];
		fAWdU[0] = Abs(rDir.x);
		fAWdU[1] = Abs(rDir.y);
		fAWdU[2] = Abs(rDir.z);
		Vec kWxD=rDir^kDiff;
		
		typename ScalarInfo<real>::TBool test[3];
		test[0]=Abs(kWxD.x)<=fAWdU[2]*bExtent[1]+fAWdU[1]*bExtent[2];
		test[1]=Abs(kWxD.y)<=fAWdU[2]*bExtent[0]+fAWdU[0]*bExtent[2];
		test[2]=Abs(kWxD.z)<=fAWdU[1]*bExtent[0]+fAWdU[0]*bExtent[1];

		return ForAny(test[0]&&test[1]&&test[2]);
	}

	template <class real,class Vec>
	bool IntersectAABB(const Vec &rDir,const Vec &rOrig,const float bExtent[3],const Vec3f &bCenter,real max) {
		Vec sDir=rDir*real(max*0.5f);
		Vec kDiff=rOrig+sDir-Vec(bCenter);

		if (ForAll( Abs(kDiff.x) > Abs(sDir.x)+bExtent[0]||
					Abs(kDiff.y) > Abs(sDir.y)+bExtent[1]||
					Abs(kDiff.z) > Abs(sDir.z)+bExtent[2]  ) )
			return 0;

		real fAWdU[3];
		fAWdU[0] = Abs(sDir.x);
		fAWdU[1] = Abs(sDir.y);
		fAWdU[2] = Abs(sDir.z);
		Vec kWxD=sDir^kDiff;
		
		typename ScalarInfo<real>::TBool test[3];
		test[0]=Abs(kWxD.x)<=fAWdU[2]*bExtent[1]+fAWdU[1]*bExtent[2];
		test[1]=Abs(kWxD.y)<=fAWdU[2]*bExtent[0]+fAWdU[0]*bExtent[2];
		test[2]=Abs(kWxD.z)<=fAWdU[1]*bExtent[0]+fAWdU[0]*bExtent[1];

		return ForAny(test[0]&&test[1]&&test[2]);
	}

	template <class real,class Vec>
	bool IntersectBBox(const BBox &box,const Vec &orig,const Vec &dir,real tMax=-1.0f) {
		real l1,l2,idir[3]={Inv(dir.x),Inv(dir.y),Inv(dir.z)};
		
		l1=idir[0]*(real(box.min.x)-orig.x);
		l2=idir[0]*(real(box.max.x)-orig.x);
		real lmin=Min(l1,l2);
		real lmax=Max(l1,l2);

		l1=idir[1]*(real(box.min.y)-orig.y);
		l2=idir[1]*(real(box.max.y)-orig.y);
		lmin=Max(Min(l1,l2),lmin);
		lmax=Min(Max(l1,l2),lmax);

		l1=idir[2]*(real(box.min.z)-orig.z);
		l2=idir[2]*(real(box.max.z)-orig.z);
		lmin=Max(Min(l1,l2),lmin);
		lmax=Min(Max(l1,l2),lmax);

		return !ForAll( lmax<real(0.0f)||lmin>lmax||(tMax>real(0.0f)&&lmin>tMax) );
	}

	template <class real,class Vec>
	bool IntersectSphere(const Vec &rDir,const Vec &rOrig,const Vec3f &sCenter,const float sRadiusSq) {
	    Vec kDiff=Vec(sCenter)-rOrig;
	    real fSqrLen=rDir|rDir;
	    real fT=(kDiff|rDir)/fSqrLen;
	    kDiff -=rDir*fT;

	    return ForAny((kDiff|kDiff) <= real(sRadiusSq));
/*		Vec dst=rOrig-sCenter;
		real B=dst|rDir;
		
		return ForAny(B*B>(dst|dst)-sRadiusSq);*/
	}
}


vector<Object*> gObjects;


void BVH::TraverseMono(const Vec3p &o,const Vec3p &d,Output<otNormal,float,u32> output,int nNode) const {
	const Node &node=nodes[nNode];

	if(!IntersectOBB<float,Vec3f>(d,o,node.obbAxis,node.obbExtent,node.obbCenter))
		return;
		
	Vec3f tOrig,tDir; {
		const Matrix<Vec4f> &m=node.invTrans;
		tOrig.x = m.x.x*o.x+m.y.x*o.y+m.z.x*o.z+m.w.x;
		tOrig.y = m.x.y*o.x+m.y.y*o.y+m.z.y*o.z+m.w.y;
		tOrig.z = m.x.z*o.x+m.y.z*o.y+m.z.z*o.z+m.w.z;
		
		tDir.x = m.x.x*d.x+m.y.x*d.y+m.z.x*d.z;
		tDir.y = m.x.y*d.x+m.y.y*d.y+m.z.y*d.z;
		tDir.z = m.x.z*d.x+m.y.z*d.y+m.z.z*d.z;
	}

	if(!IntersectBBox<float,Vec3f>(node.box,tOrig,tDir,output.dist[0]))
		return;
		

	if(!node.count) {
		gObjects[node.subNode]->TraverseMono(tOrig,tDir,output,node.id);
		return;
	}
	
	for(int n=0;n<node.count;n++)
		TraverseMono(tOrig,tDir,output,node.subNode+n);
}

Vec3f TransVec(const Vec3f &o,const Matrix<Vec4f> &m) {
	return Vec3f(
		o.x*m.x.x+o.y*m.y.x+o.z*m.z.x+m.w.x,
		o.x*m.x.y+o.y*m.y.y+o.z*m.z.y+m.w.y,
		o.x*m.x.z+o.y*m.y.z+o.z*m.z.z+m.w.z );
}

Vec3f TransNrm(const Vec3f &o,const Matrix<Vec4f> &m) {
	return Vec3f(
		o.x*m.x.x+o.y*m.y.x+o.z*m.z.x,
		o.x*m.x.y+o.y*m.y.y+o.z*m.z.y,
		o.x*m.x.z+o.y*m.y.z+o.z*m.z.z );
}

void TransRay(const Vec3q &o,const Vec3q &d,const Matrix<Vec4f> &m,Vec3q &tOrig,Vec3q &tDir) NOINLINE;

void TransRay(const Vec3q &o,const Vec3q &d,const Matrix<Vec4f> &m,Vec3q &tOrig,Vec3q &tDir) {
	tOrig.x = o.x*m.x.x+o.y*m.y.x+o.z*m.z.x+f32x4(m.w.x);
	tOrig.y = o.x*m.x.y+o.y*m.y.y+o.z*m.z.y+f32x4(m.w.y);
	tOrig.z = o.x*m.x.z+o.y*m.y.z+o.z*m.z.z+f32x4(m.w.z);

	tDir.x = d.x*m.x.x+d.y*m.y.x+d.z*m.z.x;
	tDir.y = d.x*m.x.y+d.y*m.y.y+d.z*m.z.y;
	tDir.z = d.x*m.x.z+d.y*m.y.z+d.z*m.z.z;
}

void BVH::TraverseQuad(const Vec3q &o,const Vec3q &d,Output<otNormal,f32x4,i32x4> output,int nNode) const {
	const Node &node=nodes[nNode];

//	if(!IntersectSphere<f32x4,Vec3q>(d,o,node.obbCenter,node.sRadius)) return;
//	output.stats->Skip();

	Vec3q tOrig,tDir;
	if(node.count) {
		tOrig.x=o.x+node.invTrans.w.x;
		tOrig.y=o.y+node.invTrans.w.y;
		tOrig.z=o.z+node.invTrans.w.z;
		tDir=d;
	}
	else TransRay(o,d,node.invTrans,tOrig,tDir);
	
	if(!IntersectBBox(node.box,tOrig,tDir,output.dist[0])) return;
//	Vec3f obbCenter=(node.box.max+node.box.min)*0.5f;
//	if(!IntersectAABB<f32x4,Vec3q>(tDir,tOrig,node.obbExtent,obbCenter,output.dist[0])) return;

	if(!node.count) {
		output.stats->Skip();
		gObjects[node.subNode]->TraverseQuad(tOrig,tDir,output,node.id);
		return;
	}
	
	for(int n=0;n<node.count;n++)
		TraverseQuad(tOrig,tDir,output,node.subNode+n);
}

void BVH::UpdateBox(int nNode) {
	Node &node=nodes[nNode];
	
	if(node.count) {
		UpdateBox(node.subNode);
		node.box=nodes[node.subNode].box*nodes[node.subNode].trans;
	
		for(int n=1;n<node.count;n++) {
			UpdateBox(node.subNode+n);
			node.box+=nodes[node.subNode+n].box*nodes[node.subNode+n].trans;
		}
	}
	else node.box=gObjects[node.subNode]->GetBBox();

	{
		const Matrix<Vec4f> &m=node.trans;
		node.obbCenter=(node.box.max+node.box.min)*0.5f;
		node.obbExtent[0]=node.box.Size().x*0.5f;
		node.obbExtent[1]=node.box.Size().y*0.5f;
		node.obbExtent[2]=node.box.Size().z*0.5f;
		
		node.obbCenter=TransVec(node.obbCenter,m);
		
		for(int n=0;n<3;n++)
			node.obbAxis[n]=TransNrm(Vec3f(n==0?1.0f:0.0f,n==1?1.0f:0.0f,n==2?1.0f:0.0f),m);

		if(node.obbExtent[0]<node.obbExtent[1]) {
			if(node.obbExtent[2]<node.obbExtent[0])
				node.sRadius=node.obbExtent[0]*node.obbExtent[0]+node.obbExtent[1]*node.obbExtent[1];
			else  node.sRadius=node.obbExtent[2]*node.obbExtent[2]+node.obbExtent[1]*node.obbExtent[1];
		}
		else if(node.obbExtent[2]<node.obbExtent[1])
		 	node.sRadius=node.obbExtent[0]*node.obbExtent[0]+node.obbExtent[1]*node.obbExtent[1];
		else  node.sRadius=node.obbExtent[0]*node.obbExtent[0]+node.obbExtent[2]*node.obbExtent[2];
	}
}

void BVH::UpdateGlobalTrans(int nNode,Matrix<Vec4f> *parentMat) {
	Node &node=nodes[nNode];
	node.globalTrans=parentMat?node.trans* *parentMat:node.trans;

	for(int n=0;n<node.count;n++)
		UpdateGlobalTrans(node.subNode+n,&node.globalTrans);
}

BBox BVH::GetBBox() const {
	return nodes[0].box;
}

int BVH::AddNode(const Matrix<Vec4f> &m,int sub,int count) {
	Node newNode;
	newNode.trans=m; newNode.invTrans=Inverse(m);
	newNode.subNode=sub; newNode.count=count;
	newNode.id=nodes.size();
	nodes.push_back(newNode);
	return newNode.id;
}

BVH *gBVH;
