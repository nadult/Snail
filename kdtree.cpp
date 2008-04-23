#include "kdtree.h"
#include <algorithm>
#include <omp.h>

/*
enum { res=8 };

static bool Test(bool space[res][res][res],int x,int y,int z) {
	return x<0||y<0||z<0||x>=res||y>=res||z>=res?0:space[x][y][z];
}

static bool Covered(bool space[res][res][res],int sx,int sy,int sz) {
	bool left=0,right=0,up=0,down=0,front=0,back=0;
	int x,y,z;

	x=sx; while(x>0) if(Test(space,x--,y,z)) left=1;
	x=sx; while(x<res) if(Test(space,x++,y,z)) right=1;

	y=sy; while(y>0) if(Test(space,x,y--,z)) up=1;
	y=sy; while(y<res) if(Test(space,x,y++,z)) down=1;

	z=sz; while(z>0) if(Test(space,x,y,z--)) front=1;
	z=sz; while(z<res) if(Test(space,x,y,z++)) back=1;

	return (left&&right)||(up&&down)||(front&&back);
}
*/

SlowKDTree::SlowKDTree(const vector<Object> &objs)
:objects(objs)
{
	if(objs.size()==0) {
		Convert(Vec3f(0,0,0),pMin);
		Convert(Vec3f(0,0,0),pMax);
		return;
	}


	pMin=objects[0].BoundMin(); pMax=objects[0].BoundMax();
	for(int n=0;n<objects.size();n++) {
		pMin=VMin(objects[n].BoundMin(),pMin);
		pMax=VMax(objects[n].BoundMax(),pMax);
		objects[n].SetFlag1(0);
	}
	Vec3f min,max;
	Convert(pMin,min);
	Convert(pMax,max);

	/*{
		bool space[res][res][res];
		memset(space,0,res*res*res);
		Vec3p voxSize=(pMax-pMin)/float(res);
		Vec3p iVoxSize=VInv(voxSize);

		for(uint n=0;n<objects.size();n++) {
			Object &obj=objects[n];

			Vec3p tMin=(obj.BoundMin()-pMin)*iVoxSize;
			Vec3p tMax=(obj.BoundMax()-pMin)*iVoxSize;

			int sx=tMin.x,sy=tMin.y,sz=tMin.z;
			int ex=tMax.x,ey=tMax.y,ez=tMax.z;
			for(int z=sz;z<=ez;z++)
				for(int y=sy;y<=ey;y++)
					for(int x=sx;x<=ex;x++)
						space[x][y][z]=1;
		}



		for(int y=0;y<res;y++) {
			for(int z=0;z<res;z++) {
				for(int x=0;x<res;x++) {
					if(!space[x][y][z]) if(Covered(space,x,y,z)) {
						Vec3f bMin,bMax;
						Convert(min+voxSize*Vec3p(x,y,z),bMin);
						Convert(voxSize,bMax); bMax+=bMin;

						Triangle blocker(bMin,bMax,Lerp(bMin,bMax,0.5f));
						blocker.SetFlag1(1337);
			//			objects.push_back(blocker);
			//			printf("%f %f %f    %f %f %f\n",bMin.x,bMin.y,bMin.z,bMax.x,bMax.y,bMax.z);
					}
					printf("%c",space[x][y][z]?'X':' ');
				}
				printf("\n");
			}
			printf("\n\n");
		}
	} */

	bool cutSides=1;
	int startNode=0;

	if(cutSides) {
		nodes.push_back(SlowKDNode(0,min.x,1));
		nodes.push_back(SlowKDNode());
		nodes.push_back(SlowKDNode(1,min.y,3));
		nodes.push_back(SlowKDNode());
		nodes.push_back(SlowKDNode(2,min.z,5));
		nodes.push_back(SlowKDNode());
		nodes.push_back(SlowKDNode(0,max.x,7));
		nodes.push_back(SlowKDNode(1,max.y,9));
		nodes.push_back(SlowKDNode());
		nodes.push_back(SlowKDNode(2,max.z,11));
		nodes.push_back(SlowKDNode());
		nodes.push_back(SlowKDNode());
		startNode=11;
	}
	nodes.push_back(SlowKDNode());

	for(int n=0;n<objects.size();n++) {
		objects[n].SetFlag2(1);
		nodes[startNode].objects.push_back(n);
	}

	Build(startNode,0,min,max);
}


/* Possible split position */
class PSplit
{
public:
	PSplit(float p,bool s,bool cant)
		:pos(p),start(s),cantSplit(cant) { }

	bool operator<(const PSplit& s) const
		{ return pos<s.pos; }

	float pos;
	bool start,cantSplit;
};

void SlowKDTree::Build(u32 idx,u32 level,Vec3f tMin,Vec3f tMax)
{
	uint count=0; {
		SlowKDNode &node=nodes[idx];
		for(uint n=0;n<node.objects.size();n++)
			if(objects[node.objects[n]].GetFlag1()!=1337)
				count++;
	}
	if(count<1) return;

	double size=(tMax.x-tMin.x)*(tMax.y-tMin.y)*(tMax.z-tMin.z);
	double sceneSize; { float t; Convert((pMax.x-pMin.x)*(pMax.y-pMin.y)*(pMax.z-pMin.z),t); sceneSize=t; }
	if(size/sceneSize<0.00000001) return;
	if(level>MaxLevel) return;

	int axis; float divider;
	{
		SlowKDNode &node=nodes[idx];
		vector<u32> &objs=node.objects;

		// Szukanie najwiekszego wolnego obszaru w poszczególnych wymiarach
		vector<PSplit> splits[3];
		for(int n=0;n<3;n++) splits[n].reserve(objs.size()*2);
		for(int n=0;n<objs.size();n++) {
			Object &obj=objects[objs[n]];
			Vec3f min,max;
			Convert(obj.BoundMin(),min);
			Convert(obj.BoundMax(),max);
			bool cantSplit=obj.GetFlag1()==1337;

			splits[0].push_back(PSplit(min.x,1,cantSplit));
			splits[1].push_back(PSplit(min.y,1,cantSplit));
			splits[2].push_back(PSplit(min.z,1,cantSplit));
			splits[0].push_back(PSplit(max.x,0,cantSplit));
			splits[1].push_back(PSplit(max.y,0,cantSplit));
			splits[2].push_back(PSplit(max.z,0,cantSplit));
		}
		
		const float travCost=0.2;
		const float hitTestCost=1.0;
		float noSplitCost=hitTestCost*objs.size();

		float minCost[3],dividers[3];
		float sub[3]={tMin.x,tMin.y,tMin.z},mul[3]={1.0/(tMax.x-tMin.x),1.0/(tMax.y-tMin.y),1.0/(tMax.z-tMin.z)};

		float nodeSize[3]={tMax.x-tMin.x,tMax.y-tMin.y,tMax.z-tMin.z};
		float iNodeSize=1.0/(nodeSize[0]*nodeSize[1]+nodeSize[0]*nodeSize[2]+nodeSize[1]*nodeSize[2]);
		float longestSeg[3]={0,},longestSegP[3];

#ifdef NDEBUG
		int maxThr=omp_get_max_threads();
		omp_set_num_threads(objs.size()>100?3:1);

#pragma omp parallel for
#endif
		for(int s=0;s<3;s++) {
			vector<PSplit> &split=splits[s];
			std::sort(split.begin(),split.end());
			minCost[s]=noSplitCost+1.0;

			float tNodeSize[3]={nodeSize[0],nodeSize[1],nodeSize[2]};
			int voxelsLeft=0,voxelsRight=objs.size();
			int cantSplit=0;
		
		//	float startSegLen=split[0].pos-sub[s];
		//	float endSegLen=nodeSize[s]+sub[s]-split.back().pos;
		//	if(startSegLen>endSegLen) {
		//		longestSeg[s]=startSegLen;
		//		longestSegP[s]=split[0].pos;
		//	}
		//	else {
		//		longestSeg[s]=endSegLen;
		//		longestSegP[s]=split.back().pos;
		//	}

			for(int ks=0;ks<split.size();ks++) {
				float pos=split[ks].pos;

				if(!split[ks].start) {
					if(split[ks].cantSplit) cantSplit--;
					else voxelsRight--;

				//	if(voxelsLeft+voxelsRight<=objs.size()&&ks+1<split.size()) {
				//		float seg=split[ks+1].pos-split[ks].pos;
				//		if(seg>longestSeg[s]) {
				//			longestSeg[s]=seg;
				//			longestSegP[s]=ks>0?split[ks].pos:split[ks+1].pos;
				//		}
				//	}
				}
				float posInNode=(pos-sub[s])*mul[s];

				if(!cantSplit) if(posInNode>0.0&&posInNode<1.0) {
					tNodeSize[s]=nodeSize[s]*posInNode;
					float leftSize= (tNodeSize[0]*tNodeSize[1]+tNodeSize[0]*tNodeSize[2]+tNodeSize[1]*tNodeSize[2])*iNodeSize;
					tNodeSize[s]=nodeSize[s]*(1.0-posInNode);
					float rightSize=(tNodeSize[0]*tNodeSize[1]+tNodeSize[0]*tNodeSize[2]+tNodeSize[1]*tNodeSize[2])*iNodeSize;

					float splitCost=travCost+hitTestCost*(leftSize*float(voxelsLeft)+rightSize*float(voxelsRight));

					if(splitCost<minCost[s]) {
						minCost[s]=splitCost;
						dividers[s]=pos;
					}
				}

				if(split[ks].start) {
					if(split[ks].cantSplit) cantSplit++;
					else voxelsLeft++;
				}
			}
		}

#ifdef NDEBUG
		omp_set_num_threads(maxThr);
#endif

		/*for(int s=0;s<3;s++) {
			double nodeSize=(&(tMax-tMin).x)[s];
			if(longestSeg[s]>nodeSize*0.1) {
				
				dividers[s]=longestSegP[s];
				minCost[s]=1.0-longestSeg[s]/nodeSize;
			//	printf("%f\n",minCost[s]);
			}
		}*/

		axis=minCost[0]<minCost[2]?0:2;
		if(minCost[1]<minCost[axis]) axis=1;
		if(noSplitCost<minCost[axis]) return;

		divider=dividers[axis];

		node.type=(SlowKDNode::Type)axis;
		node.pos=divider;
		node.child=nodes.size();
	}

	nodes.push_back(SlowKDNode());
	nodes.push_back(SlowKDNode());

	{ // Przerzucanie obiektow
		SlowKDNode &left=nodes[nodes.size()-2],&right=nodes[nodes.size()-1],&node=nodes[idx];
		vector<u32> midObjs;
		for(int n=0;n<node.objects.size();n++) {
			Object &obj=objects[node.objects[n]];
			Vec3f min,max;

			Convert(obj.BoundMin(),min);
			Convert(obj.BoundMax(),max);
			float pMin=((float*)&min)[axis];
			float pMax=((float*)&max)[axis];

			if(pMin<divider&&pMax>divider)
				obj.SetFlag2(0);

			bool added=0;
			if(pMin<divider) { left.objects.push_back(node.objects[n]); added=1; }
			if(pMax>divider) { right.objects.push_back(node.objects[n]); added=1; }
			if(!added) midObjs.push_back(node.objects[n]);
		}

		for(int n=0;n<midObjs.size();n++) {
	//		if(left.objects.size()<right.objects.size())
				left.objects.push_back(midObjs[n]);
	//		else
	//			right.objects.push_back(midObjs[n]);
		}
		node.objects.clear();
	}
	Vec3f newTMax(tMax),newTMin(tMin);
	((float*)&newTMax)[axis]=divider;
	((float*)&newTMin)[axis]=divider;

	u32 left=nodes.size()-2,right=nodes.size()-1;
	if(level==0) {
		Build(left,level+1,tMin,newTMax);
		Build(right,level+1,newTMin,tMax);
	}
	else {		
		Build(left,level+1,tMin,newTMax);
		Build(right,level+1,newTMin,tMax);
	}
}

KDTree::KDTree(const vector<Object> &objects) {
	SlowKDTree slowkd(objects);
	Build(slowkd);
}

KDTree::KDTree(const SlowKDTree &tree) {
	Build(tree);
}

void KDTree::Build(const SlowKDTree &tree) {
	pMin=tree.pMin; pMax=tree.pMax;

	nodes.resize(tree.nodes.size());
	objects=tree.objects;

	for(u32 n=0;n<nodes.size();n++) {
		const SlowKDNode &sNode=tree.nodes[n];
		KDNode &node=nodes[n];

		if(sNode.type==SlowKDNode::T_LEAF) {
			node.SetLeaf(objectIds.size(),sNode.objects.size());
			for(u32 i=0;i<sNode.objects.size();i++) {
				if(objects[sNode.objects[i]].GetFlag1()!=1337)
				objectIds.push_back(sNode.objects[i]);
			}
		}
		else node.SetNode(sNode.type,sNode.pos,sNode.child-n);
		int a=0;
	}

	if(!nodes.size()) {
		KDNode tNode; tNode.SetLeaf(0,0);
		nodes.push_back(tNode);
	}
}

void KDTree::PrintInfo() const {
	if(!Test()) {
		printf("Test not passed!\n");
	}

	int full=0,notFull=0;
	for(int n=0;n<objects.size();n++) {
		if(objects[n].GetFlag2()) full++;
		else notFull++;
	}

	printf("Objects: %7d    Nodes: %7d    Single node objects: %.2f%%\n",full+notFull,nodes.size(),100.0*double(full)/double(full+notFull));
}

KDTree::~KDTree()
{
}

bool KDTree::TestNode(Vec3f min,Vec3f max,int n) const {
	const KDNode &node=nodes[n];
	bool left,right;
	return 1;

	switch(node.Axis()) {
	case 0:
		if(node.Pos()<min.x||node.Pos()>max.x) goto ERR;
		left=TestNode(min,Vec3f(node.Pos(),max.y,max.z),n+node.ChildDist());
		right=TestNode(Vec3f(node.Pos(),min.y,min.z),max,n+node.ChildDist()+1);
		return left&&right;
	case 1:
		if(node.Pos()<min.y||node.Pos()>max.y) goto ERR;
		left=TestNode(min,Vec3f(max.x,node.Pos(),max.z),n+node.ChildDist());
		right=TestNode(Vec3f(min.x,node.Pos(),min.z),max,n+node.ChildDist()+1);
		return left&&right;
	case 2:
		if(node.Pos()<min.z||node.Pos()>max.z) goto ERR;
		left=TestNode(min,Vec3f(max.x,max.y,node.Pos()),n+node.ChildDist());
		right=TestNode(Vec3f(min.x,min.y,node.Pos()),max,n+node.ChildDist()+1);
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

