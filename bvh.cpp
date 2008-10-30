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
		SortObjects(const BVHBuilder &b,int ax) :builder(b),axis(ax) { }
		
		bool operator()(int a,int b) const {
			const BBox &ba=builder.objects[builder.instances[a].objId].bBox*builder.instances[a].trans;
			const BBox &bb=builder.objects[builder.instances[b].objId].bBox*builder.instances[b].trans;
			return (&ba.Center().x)[axis]<(&bb.Center().x)[axis];
		}
		
		int axis;
		const BVHBuilder &builder;
	};
	
};

void BVH::FindSplit(int nNode,const BVHBuilder &builder,vector<int> &indices) {
	if(indices.size()<=2) {
		nodes[nNode].count=indices.size();
		nodes[nNode].subNode=nodes.size();
		
		for(int n=0;n<indices.size();n++) {
			const BVHBuilder::Instance &inst=builder.instances[indices[n]];
			const BVHBuilder::Obj &obj=builder.objects[inst.objId];
			
			Node newNode;
			newNode.bBox=obj.bBox*inst.trans;
			newNode.trans=obj.preTrans*inst.trans;
			newNode.invTrans=Inverse(newNode.trans);
			newNode.optBBox=obj.optBBox;

			newNode.subNode=inst.objId;
			newNode.count=0;
			newNode.id=nodes.size();
			nodes.push_back(newNode);
		}
	}
	else {
		int axis; {
			BBox box=(builder.objects[builder.instances[indices[0]].objId].bBox)*builder.instances[indices[0]].trans;
			for(int n=1;n<indices.size();n++) 
				box+=(builder.objects[builder.instances[indices[n]].objId].bBox)*builder.instances[indices[n]].trans;
				
			Vec3f bSize=box.Size();
			axis=bSize.x>bSize.y?0:1;
			if(bSize.z>(&bSize.x)[axis]) axis=2;
		}
		
		std::sort(indices.begin(),indices.end(),SortObjects(builder,axis));
		nodes[nNode].count=2;
		nodes[nNode].subNode=nodes.size();
		
		vector<int> left,right;
		for(int n=0;n<indices.size();n++)
			(n<indices.size()/2?left:right).push_back(indices[n]);
			
		int leftId =AddNode(-1,left.size());
		int rightId=AddNode(-1,right.size());
		
		FindSplit(leftId,builder,left);
		FindSplit(rightId,builder,right);
	}
}

void BVH::Build(const BVHBuilder &builder) {
	nodes.clear();
	objects.clear();
	
	for(int n=0;n<builder.objects.size();n++)
		objects.push_back(builder.objects[n].object);
	
	vector<int> indices;
	for(int n=0;n<builder.instances.size();n++)
		indices.push_back(n);
	
	AddNode(1,indices.size());
	FindSplit(0,builder,indices);
	
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
	UpdateTD(0,~0);
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

void BVH::UpdateTD(int nNode,int nParent) {
	Node &node=nodes[nNode];
	node.globalTrans=nParent!=~0?node.trans*nodes[nParent].trans:node.trans;
	for(int n=0;n<node.count;n++)
		UpdateTD(node.subNode+n,nNode);
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
