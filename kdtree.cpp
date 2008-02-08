#include "stdafx.h"
#include "rtracer.h"
#include <algorithm>

SlowKDTree::SlowKDTree(const vector<Object> &objs)
:objects(objs)
{
	nodes.push_back(SlowKDNode());
	if(objs.size()==0) {
		Convert(Vec3f(0,0,0),pMin);
		Convert(Vec3f(0,0,0),pMax);
		return;
	}

	for(int n=0;n<objects.size();n++) {
		objects[n].fullInNode=1;
		nodes[0].objects.push_back(n);
	}

	pMin=objs[0].BoundMin(); pMax=objs[0].BoundMax();
	for(int n=0;n<objs.size();n++) {
		pMin=Min(objs[n].BoundMin(),pMin);
		pMax=Max(objs[n].BoundMax(),pMax);
	}
	Vec3f min,max;
	Convert(pMin,min);
	Convert(pMax,max);
	Build(0,0,min,max);
}


/* Possible split position */
class PSplit
{
public:
	PSplit(float p,bool s)
		:pos(p),start(s) { }

	bool operator<(const PSplit& s) const
		{ return pos<s.pos; }

	float pos;
	bool start;
};


void SlowKDTree::Build(u32 idx,u32 level,Vec3f tMin,Vec3f tMax)
{
	if(nodes[idx].objects.size()<=0) { nodes[idx].notEmpty=0; return; }

	if(level>MaxLevel) {
		nodes[idx].pMin=tMin;
		nodes[idx].pMax=tMax;
		return;
	}

	int axis; float divider;
	{
		SlowKDNode &node=nodes[idx];
		vector<u32> &objs=node.objects;

		// Szukanie najwiêkszego wolnego obszaru w poszczególnych wymiarach
		vector<PSplit> splits[3];
		for(int n=0;n<3;n++) splits[n].reserve(objs.size()*2);
		for(int n=0;n<objs.size();n++) {
			Object &obj=objects[objs[n]];
			Vec3f min,max;
			Convert(obj.BoundMin(),min);
			Convert(obj.BoundMax(),max);

			splits[0].push_back(PSplit(min.X(),1));
			splits[1].push_back(PSplit(min.Y(),1));
			splits[2].push_back(PSplit(min.Z(),1));
			splits[0].push_back(PSplit(max.X(),0));
			splits[1].push_back(PSplit(max.Y(),0));
			splits[2].push_back(PSplit(max.Z(),0));
		}
		
		const double travCost=0.14;
		const double hitTestCost=1.0;
		double noSplitCost=hitTestCost*objs.size();

		double minCost[3],dividers[3];
		double sub[3]={tMin.x,tMin.y,tMin.z},mul[3]={1.0/(tMax.x-tMin.x),1.0/(tMax.y-tMin.y),1.0/(tMax.z-tMin.z)};

		double nodeSize[3]={tMax.x-tMin.x,tMax.y-tMin.y,tMax.z-tMin.z};
		double iNodeSize=1.0/(nodeSize[0]*nodeSize[1]+nodeSize[0]*nodeSize[2]+nodeSize[1]*nodeSize[2]);
//		double longestSeg[3]={0,},longestSegP[3];

#pragma omp parallel for
		for(int s=0;s<3;s++) {
			vector<PSplit> &split=splits[s];
			std::sort(split.begin(),split.end());
			minCost[s]=noSplitCost+1.0;

			double tNodeSize[3]={nodeSize[0],nodeSize[1],nodeSize[2]};
			double voxelsLeft=0,voxelsRight=objs.size();
		
/*			if(split[0].pos-sub[s]>0) {
				longestSeg[s]=split[0].pos-sub[s];
				longestSegP[s]=split[0].pos;
			}
			int opened=0;*/

			for(int ks=0;ks<split.size();ks++) {
				double pos=split[ks].pos;
				if(!split[ks].start) { voxelsRight--; /*opened--;*/ }
				double posInNode=(pos-sub[s])*mul[s];

				if(posInNode>0.0&&posInNode<1.0) {
					tNodeSize[s]=nodeSize[s]*posInNode;
					double leftSize= (tNodeSize[0]*tNodeSize[1]+tNodeSize[0]*tNodeSize[2]+tNodeSize[1]*tNodeSize[2])*iNodeSize;
					tNodeSize[s]=nodeSize[s]*(1.0-posInNode);
					double rightSize=(tNodeSize[0]*tNodeSize[1]+tNodeSize[0]*tNodeSize[2]+tNodeSize[1]*tNodeSize[2])*iNodeSize;

					double splitCost=travCost+hitTestCost*(leftSize*voxelsLeft+rightSize*voxelsRight);

					if(splitCost<minCost[s]) {
						minCost[s]=splitCost;
						dividers[s]=pos;
					}
				}

				if(split[ks].start) { voxelsLeft++; /*opened++;*/ }
		/*		if(opened==0&&ks+1<split.size()) {
					double seg=split[ks+1].pos-split[ks].pos;
					if(seg>longestSeg[s]) {
						longestSeg[s]=seg;
						longestSegP[s]=ks>0?split[ks].pos:split[ks+1].pos;
					}
				}*/
			}
		}

		/*
		for(int s=0;s<3;s++) {
			double nodeSize=(&(tMax-tMin).x)[s];
			if(longestSeg[s]>nodeSize*0.1) {
				dividers[s]=longestSegP[s];
				minCost[s]=1.0-longestSeg[s]/nodeSize;
				//printf("%f\n",minCost[s]);
			}
		}*/

		axis=minCost[0]<minCost[2]?0:2;
		if(minCost[1]<minCost[axis]) axis=1;
		if(noSplitCost<minCost[axis]) {
			nodes[idx].pMin=tMin;
			nodes[idx].pMax=tMax;
			return;
		}

		divider=dividers[axis];

		node.type=(SlowKDNode::Type)axis;
		node.pos=divider;
		node.child=nodes.size();
	}

	nodes.push_back(SlowKDNode());
	nodes.push_back(SlowKDNode());

	{ // Przerzucanie obiektow
		SlowKDNode &left=nodes[nodes.size()-2],&right=nodes[nodes.size()-1],&node=nodes[idx];
		for(int n=0;n<node.objects.size();n++) {
			Object &obj=objects[node.objects[n]];
			Vec3f min,max;

			Convert(obj.BoundMin(),min);
			Convert(obj.BoundMax(),max);
			float pMin=((float*)&min)[axis];
			float pMax=((float*)&max)[axis];

			if(pMin<divider&&pMax>divider)
				obj.fullInNode=0;

			if(pMin<divider) left.objects.push_back(node.objects[n]);
			if(pMax>divider) right.objects.push_back(node.objects[n]);
		}
		node.objects.clear();
	}
	Vec3f newTMax(tMax),newTMin(tMin);
	((float*)&newTMax)[axis]=divider;
	((float*)&newTMin)[axis]=divider;

	u32 left=nodes.size()-2,right=nodes.size()-1;
	Build(left,level+1,tMin,newTMax);
	Build(right,level+1,newTMin,tMax);
}

void SlowKDTree::Draw(Image &img,Vec3f minP,Vec3f maxP,
					  const Camera &cam,u32 n) const
{
/*	const SlowKDNode &node=nodes[n];
	float mov=100.0f,scl=2.0f;

#define PIX(x,z,r,g,b)	img.Pixel(mov+(x)*scl,mov+(z)*scl,r,g,b);

	if(n==0) {
		for(int z=minP.z*scl,ez=maxP.z*scl;z<ez;z++) img.Pixel(mov+minP.x*scl,mov+z,0,255,0);
		for(int z=minP.z*scl,ez=maxP.z*scl;z<ez;z++) img.Pixel(mov+maxP.x*scl,mov+z,0,255,0);
		for(int x=minP.x*scl,ex=maxP.x*scl;x<ex;x++) img.Pixel(mov+x,mov+minP.z*scl,0,255,0);
		for(int x=minP.x*scl,ex=maxP.x*scl;x<ex;x++) img.Pixel(mov+x,mov+maxP.z*scl,0,255,0);

		img.Pixel(mov+cam.pos.x*scl,mov+cam.pos.z*scl,255,255,255);
		img.Pixel(mov+cam.pos.x*scl,mov+cam.pos.z*scl-1,255,255,255);
		img.Pixel(mov+cam.pos.x*scl-1,mov+cam.pos.z*scl,255,255,255);
		img.Pixel(mov+cam.pos.x*scl-1,mov+cam.pos.z*scl-1,255,255,255);

		img.Pixel(mov+(cam.pos.x+cam.front.x*0.5f)*scl,mov+(cam.pos.z+cam.front.z*0.5f)*scl-1,255,255,255);
		img.Pixel(mov+(cam.pos.x+cam.front.x)*scl,mov+(cam.pos.z+cam.front.z)*scl-1,255,255,255);
	}
	
	switch(node.type) {
	case SlowKDNode::T_X: {
		int sz=mov+minP.z*scl,ez=mov+maxP.z*scl,x=mov+node.pos*scl;
		for(int z=sz;z<ez;z++) img.Pixel(x,z,255,0,0);

		Draw(img,minP,Vec3f(node.pos,maxP.y,maxP.z),cam,node.child);
		Draw(img,Vec3f(node.pos,minP.y,minP.z),maxP,cam,node.child+1);
		break; }
	case SlowKDNode::T_Y: {
		Draw(img,minP,Vec3f(maxP.x,node.pos,maxP.z),cam,node.child);
		Draw(img,Vec3f(minP.x,node.pos,minP.z),maxP,cam,node.child+1);	
		break; }
	case SlowKDNode::T_Z: {
		int sx=mov+minP.x*scl,ex=mov+maxP.x*scl,z=mov+node.pos*scl;
		for(int x=sx;x<ex;x++) img.Pixel(x,z,0,0,255);

		Draw(img,minP,Vec3f(maxP.x,maxP.y,node.pos),cam,node.child);
		Draw(img,Vec3f(minP.x,minP.y,node.pos),maxP,cam,node.child+1);
		break; }
	default:
		break;
	}	

	for(int n=0;n<node.objects.size();n++) {
		const Object &s=node.objects[n];

		PIX(s.pos.x,s.pos.z,255,255,255);
		PIX(s.pos.x-s.rad,s.pos.z,255,255,255);
		PIX(s.pos.x+s.rad,s.pos.z,255,255,255);
		PIX(s.pos.x,s.pos.z-s.rad,255,255,255);
		PIX(s.pos.x,s.pos.z+s.rad,255,255,255);
	}
#undef PIX*/
}

static SSEPVec3 buffer[1000000];

KDTree::KDTree(const SlowKDTree &tree)
{
	pMin=tree.pMin; pMax=tree.pMax;

	nodes.resize(tree.nodes.size());
	objects=tree.objects;

	nodeMinMax=buffer;//(SSEPVec3*)_aligned_malloc(sizeof(SSEPVec3)*nodes.size()*2,16);

	for(u32 n=0;n<nodes.size();n++) {
		const SlowKDNode &sNode=tree.nodes[n];
		KDNode &node=nodes[n];

		if(sNode.type==SlowKDNode::T_LEAF) {
			Convert(sNode.pMin,nodeMinMax[n*2+0]);
			Convert(sNode.pMax,nodeMinMax[n*2+1]);

			node.SetLeaf(objectIds.size(),sNode.objects.size());
			for(u32 i=0;i<sNode.objects.size();i++)
				objectIds.push_back(sNode.objects[i]);
		}
		else node.SetNode(sNode.type,sNode.pos,sNode.child-n);
		int a=0;
	}
}
KDTree::~KDTree()
{
//	_aligned_free(nodeMinMax);
}

bool KDTree::TestNode(Vec3f min,Vec3f max,int n) const {
	const KDNode &node=nodes[n];
	bool left,right;

	switch(node.Axis()) {
	case 0:
		if(node.Pos()<min.X()||node.Pos()>max.X()) goto ERR;
		left=TestNode(min,Vec3f(node.Pos(),max.Y(),max.Z()),n+node.ChildDist());
		right=TestNode(Vec3f(node.Pos(),min.Y(),min.Z()),max,n+node.ChildDist()+1);
		return left&&right;
	case 1:
		if(node.Pos()<min.Y()||node.Pos()>max.Y()) goto ERR;
		left=TestNode(min,Vec3f(max.X(),node.Pos(),max.Z()),n+node.ChildDist());
		right=TestNode(Vec3f(min.X(),node.Pos(),min.Z()),max,n+node.ChildDist()+1);
		return left&&right;
	case 2:
		if(node.Pos()<min.Z()||node.Pos()>max.Z()) goto ERR;
		left=TestNode(min,Vec3f(max.X(),max.Y(),node.Pos()),n+node.ChildDist());
		right=TestNode(Vec3f(min.X(),min.Y(),node.Pos()),max,n+node.ChildDist()+1);
		return left&&right;
	case 3:
		return 1;
	}

ERR:
	printf("Error in node #%d\n",n);
	return 0;
}

bool KDTree::Test() const {
	Vec3f min,max;
	Convert(pMin,min); Convert(pMax,max);
	return TestNode(min,max,0);
}

void KDTree::Prepare() const {
	for(int n=0;n<objects.size();n++)
		objects[n].lastVisit=0;
}
