#include "bvh.h"
#include "bihtree.h"
#include <algorithm>


int BVHBuilder::AddObject(PObject object,const Matrix<Vec4f> &preTrans,const BBox &box,const OptBBox &optBBox) {
	Obj obj;
	obj.object=object;
	obj.preTrans=preTrans;
	obj.bBox=box;
	obj.optBBox=optBBox;
	obj.id=objects.size();
	objects.push_back(obj);
	
	return obj.id;
}

int BVHBuilder::AddInstance(int objId,const Matrix<Vec4f> &trans) {
	Instance inst;
	inst.trans=trans;
	inst.objId=objId;
	inst.id=instances.size();
	instances.push_back(inst);
	
	return inst.id;
}

namespace {
	
	struct SortObjects {
		SortObjects(const vector<float> &vals) :values(vals) { }
		
		bool operator()(int a,int b) const { return values[a]<values[b]; }
		
		const vector<float> &values;
	};
	
	/*
	float BoxPointDistanceSq(const BBox &box,const Vec3f &point) const {
		Vec3f center=box.Center(),size=box.Size()*0.5f;
		float diff[3]={point.x-center.x,point.y-center.y,point.z-center.z};
		float length[3]={size.x,size.y,size.z};
		float p[3]={point.x,point.y,point.z};

		float distance = 0.0;
		float delta;

		for( int i=0; i<3; i++ )
			if ( diff[i]<-length[i] ) {
				delta=diff[i]+length[i];
				distance+=delta*delta;
				diff[i]=-length[i];
			}
			else if (diff[i]>length[i] ) {
				delta=diff[i]-length[i];
				distance+=delta*delta;
				diff[i]=length[i];
			}

		return distance;
	} */
	
};

void BVH::FindSplit(int nNode,BBox sceneBox,const BVHBuilder &builder,vector<int> &indices,int first,int count,
					const vector<BBox> &instBBoxes,const vector<float> centers[3],int depth) {
	if(count<=2) {
		nodes[nNode].count=count;
		nodes[nNode].subNode=nodes.size();
		
		for(int n=0;n<count;n++) {
			const BVHBuilder::Instance &inst=builder.instances[indices[first+n]];
			const BVHBuilder::Obj &obj=builder.objects[inst.objId];
			
			Node newNode;
			newNode.bBox=obj.bBox*inst.trans;
			newNode.trans=obj.preTrans*inst.trans;
			newNode.invTrans=Inverse(newNode.trans);
			newNode.optBBox=OptBBox(obj.optBBox.GetBBox(),obj.optBBox.GetTrans()*newNode.trans);

			newNode.subNode=inst.objId;
			newNode.count=0;
			newNode.id=nodes.size();
			nodes.push_back(newNode);
		}
	}
	else {
		int axis; {		
			Vec3f bSize=sceneBox.Size();
			axis=bSize.x>bSize.y?0:1;
			if(bSize.z>(&bSize.x)[axis]) axis=2;
		}
		BBox leftBox=sceneBox,rightBox=sceneBox;
		float split=(&sceneBox.Center().x)[axis];
		(&leftBox .max.x)[axis]=split;
		(&rightBox.min.x)[axis]=split;
		
		std::sort(&indices[first],&indices[first+count],SortObjects(centers[axis]));
			
		int lCount; {
			int l=first,r=first+count-1;
			while(l<r) {
				int m=(l+r)/2;
				if((&instBBoxes[indices[m]].Center().x)[axis]<split) l=m+1;
				else r=m;
			}
			int rStart=(&instBBoxes[indices[r]].Center().x)[axis]<split?r+1:r;
			
			lCount=rStart-first;
		}
		int rCount=count-lCount;

		if(lCount&&rCount) {
			nodes[nNode].count=2;
			nodes[nNode].subNode=nodes.size();
			nodes[nNode].divAxis=axis;
			
			int leftId =AddNode(-1,lCount);
			int rightId=AddNode(-1,count-lCount);
			
			FindSplit(leftId,leftBox,builder,indices,first,lCount,instBBoxes,centers,depth+1);
			FindSplit(rightId,rightBox,builder,indices,first+lCount,count-lCount,instBBoxes,centers,depth+1);
		}
		else {
			if(lCount) FindSplit(nNode,leftBox,builder,indices,first,count,instBBoxes,centers,depth+1);
			if(rCount) FindSplit(nNode,rightBox,builder,indices,first,count,instBBoxes,centers,depth+1);
		}
	}
	
	maxDepth=Max(maxDepth,depth);
}

void BVH::Build(const BVHBuilder &builder) {
	nodes.reserve(builder.objects.size()*2);
	nodes.clear();
	
	objects.resize(builder.objects.size());
	for(int n=0;n<builder.objects.size();n++)
		objects[n]=builder.objects[n].object;
	
	vector<BBox> instBBoxes(builder.instances.size());
	vector<int> indices(builder.instances.size());
	
	
	for(int n=0;n<builder.instances.size();n++) indices[n]=n;
	for(int n=0;n<instBBoxes.size();n++) {
		const BVHBuilder::Instance &inst=builder.instances[n];
		instBBoxes[n]=(builder.objects[inst.objId].bBox)*inst.trans;
	}

	BBox sceneBox=instBBoxes[0];
	for(int n=1;n<instBBoxes.size();n++) sceneBox+=instBBoxes[n];
	
	vector<float> centers[3];
	for(int k=0;k<3;k++) centers[k].resize(instBBoxes.size());
	for(int n=0;n<instBBoxes.size();n++) {
		Vec3f center=instBBoxes[n].Center();
		centers[0][n]=center.x;
		centers[1][n]=center.y;
		centers[2][n]=center.z;
	}
	
	maxDepth=0;
		
	AddNode(1,indices.size());
	FindSplit(0,sceneBox,builder,indices,0,indices.size(),instBBoxes,centers,0);
	
	Update();
}

float BoxPointDistanceSq(const BBox &box,const Vec3f &point,const Vec3f &dir) {
	Vec3f min=box.min,max=box.max;

	/*{	
		Vec3f points[8]={
			Vec3f(min.x,min.y,min.z),
			Vec3f(min.x,min.y,max.z),
			Vec3f(min.x,max.y,min.z),
			Vec3f(min.x,max.y,max.z),
			Vec3f(max.x,min.y,min.z),
			Vec3f(max.x,min.y,max.z),
			Vec3f(max.x,max.y,min.z),
			Vec3f(max.x,max.y,max.z) };
		float dist=1.0f/0.0f;
		for(int n=0;n<8;n++)
			dist+=LengthSq(points[n]-point);
		return dist;
	}*/
	{
		floatq mmy(min.y,min.y,max.y,max.y);
		floatq mmz(min.z,max.z,min.z,max.z);
		
		Vec3q b[2]={ Vec3q(min.x,mmy,mmz),Vec3q(max.x,mmy,mmz) };
		Vec3q tp(point.x,point.y,point.z);

		b[0]-=tp; b[1]-=tp;	
		floatq dist=LengthSq(b[0])+LengthSq(b[1]);
			
		return dist[0]+dist[1]+dist[2]+dist[3];
	}
}

void BVH::Update() {
	UpdateBU(0,~0);
}

void BVH::UpdateBU(int nNode,int nParent) {
	Node &node=nodes[nNode];
	
	if(node.count) {
		UpdateBU(node.subNode,nNode);
		node.bBox=nodes[node.subNode].bBox;
	
		for(int n=1;n<node.count;n++) {
			UpdateBU(node.subNode+n,nNode);
			node.bBox+=nodes[node.subNode+n].bBox;
		}
	}
	else {
	}
	/*{
		node.obbCenter=node.bBox.Center();
		float obbExtent[3];
		obbExtent[0]=node.bBox.Size().x*0.5f;
		obbExtent[1]=node.bBox.Size().y*0.5f;
		obbExtent[2]=node.bBox.Size().z*0.5f;
		
		if(obbExtent[0]<obbExtent[1]) {
			if(obbExtent[2]<obbExtent[0])
				node.sRadius=obbExtent[0]*obbExtent[0]+obbExtent[1]*obbExtent[1];
			else  node.sRadius=obbExtent[2]*obbExtent[2]+obbExtent[1]*obbExtent[1];
		}
		else if(obbExtent[2]<obbExtent[1])
		 	node.sRadius=obbExtent[0]*obbExtent[0]+obbExtent[1]*obbExtent[1];
		else  node.sRadius=obbExtent[0]*obbExtent[0]+obbExtent[2]*obbExtent[2];
	}*/
}

BBox BVH::GetBBox() const {
	return nodes[0].bBox;
}

int BVH::AddNode(int sub,int count) {
	Node newNode;
	newNode.trans=Identity<>();
	newNode.invTrans=Inverse(Identity<>());
	newNode.subNode=sub;
	newNode.count=count;
	
	newNode.id=nodes.size();
	nodes.push_back(newNode);
	
	return newNode.id;
}
