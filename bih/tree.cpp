#include <algorithm>
#include <stdio.h>
#include "bih/tree.h"
#include <iostream>
#include <ostream>
#include <queue>

namespace bih {

	namespace {

		inline uint MaxAxis(Vec3p size) {
			uint axis=0;
			float s=size.x;
			if(size.y>s) { s=size.y; axis=1; }
			if(size.z>s) axis=2;
			return axis;
		}
	
		class PSplit
		{
		public:
			INLINE PSplit() { }
			PSplit(float ta,float tb,bool s) :a(ta),b(tb),start(s) { pos=start?a:b; }
			bool operator<(const PSplit& s) const { return pos==s.pos?a==s.a?b<s.b:a<s.a:pos<s.pos; }
			bool operator==(const PSplit& s) const { return pos==s.pos&&a==s.a&&b==s.b; }

			float pos,a,b;
			int start;
		};

	}

	void FindSplit(const vector<Index> &indices,const Vec3f &min,const Vec3f &max,int &outAxis,float &outSplit) {
		int axis=MaxAxis(max-min);
		float cmin=(&min.x)[axis],cmax=(&max.x)[axis];
		float split=Lerp(cmin,cmax,0.5f);
		outAxis=axis;
		outSplit=split;
	}

	bool SAH(const vector<Index> &indices,const Vec3f &min,const Vec3f &max,int &outAxis,float &outSplit) {
		enum { pickLongestAxis=0 };
		int axis=MaxAxis(max-min);

		const float travCost=0.1;
		const float hitTestCost=1.0;
		const float noSplitCost=hitTestCost*indices.size();

		float minCost[3],dividers[3];
		const float sub[3]={min.x,min.y,min.z},mul[3]={1.0f/(max.x-min.x),1.0f/(max.y-min.y),1.0f/(max.z-min.z)};

		const float nodeSize[3]={max.x-min.x,max.y-min.y,max.z-min.z};
		const float iNodeSize=1.0/(nodeSize[0]*nodeSize[1]+nodeSize[0]*nodeSize[2]+nodeSize[1]*nodeSize[2]);

		int startS=pickLongestAxis?axis:0,endS=pickLongestAxis?axis+1:3; {
			int nSplits=indices.size()*2;
			vector<PSplit> splits(nSplits);

			for(int s=startS;s<endS;s++) {
				for(int n=0;n<indices.size();n++) {
					Vec3f min=indices[n].min,max=indices[n].max;
					splits[n*2+0]=PSplit((&min.x)[s],(&max.x)[s],0);
					splits[n*2+1]=PSplit((&min.x)[s],(&max.x)[s],1);
				}

				std::sort(splits.begin(),splits.end());
				minCost[s]=1.0f/0.0f;

				float tNodeSize[3]={nodeSize[0],nodeSize[1],nodeSize[2]};
				int left=0,right=indices.size();

				for(int ks=0;ks<nSplits;ks++) {
					float pos=splits[ks].pos;

					if(!splits[ks].start) right--;
					float posInNode=(pos-sub[s])*mul[s];

					if(posInNode>0.0&&posInNode<1.0) {
						tNodeSize[s]=nodeSize[s]*posInNode;
						float leftSize= (tNodeSize[0]*(tNodeSize[1]+tNodeSize[2])+tNodeSize[1]*tNodeSize[2]);
						tNodeSize[s]=nodeSize[s]*(1.0-posInNode);
						float rightSize=(tNodeSize[0]*(tNodeSize[1]+tNodeSize[2])+tNodeSize[1]*tNodeSize[2]);

						float splitCost=leftSize*float(left)+rightSize*float(right);

						if(splitCost<minCost[s]) {
							minCost[s]=splitCost;
							dividers[s]=pos;
						}
					}
					if(splits[ks].start) left++;
				}
			}
		}

		for(int k=startS;k<endS;k++)
			minCost[k]=travCost+hitTestCost*minCost[k]*iNodeSize;

		if(!pickLongestAxis) {
			axis=minCost[0]<minCost[2]?0:2;
			if(minCost[1]<minCost[axis]) axis=1;
		}
		if(noSplitCost<minCost[axis]) return 0;

		outAxis=axis;
		outSplit=dividers[axis];
		return 1;
	}

	template <class ElementContainer>
	Tree<ElementContainer>::Tree(const ElementContainer &objs) { Construct(objs); }

	template <class ElementContainer>
	void Tree<ElementContainer>::Construct(const ElementContainer &objs) {
		elements=objs;

		if(!elements.size()) {
			nodes.push_back(Node());
			nodes[0].clip[0]=-1.0f/0.0f;
			nodes[0].clip[1]=1.0f/0.0f;
			nodes[0].val[0]=nodes[0].val[1]=0;
			return;
		}

		pMin=elements.BoundMin(0);
		pMax=elements.BoundMax(0);
		maxLevel=0;

		Vec3p sumSize(0,0,0);
		for(uint n=1;n<elements.size();n++) {
			Vec3p min=elements.BoundMin(n),max=elements.BoundMax(n);
			sumSize+=max-min;

			pMin=VMin(pMin,min);
			pMax=VMax(pMax,max);
		}
		nodes.push_back(Node());
		
		avgSize=sumSize.x+sumSize.y+sumSize.z;
		avgSize/=3.0*elements.size();

	//	printf("."); fflush(stdout);
		vector<Index> indices; indices.reserve(elements.size()*16);

		for(int n=0;n<elements.size();n++)
			indices.push_back(Index(n,elements.BoundMin(n),elements.BoundMax(n),1.0f));

		vector<u32> parents; parents.push_back(0);

		Build(indices,parents,0,pMin,pMax,0,1);
	//	OptimizeBFS();
	}

	template <class ElementContainer>
	void Tree<ElementContainer>::Serialize(Serializer &sr) {
		sr & nodes & elements;
		sr & avgSize & pMin & pMax;
		sr & objectId & maxLevel;
	}

	template <class ElementContainer>
	void Tree<ElementContainer>::PrintInfo() const {
		double nodeBytes=nodes.size()*sizeof(Node);
		double objBytes=elements.mem_size();

		printf("Elems:  %6.2fMB\n",objBytes*0.000001);
		printf("Nodes: %8d * %2d = %6.2fMB\n",nodes.size(),sizeof(Node),nodeBytes*0.000001);
		printf("~ %.0f bytes per triangle\n",double(nodeBytes+objBytes)/double(elements.size()));
		printf("Levels: %d\n\n",maxLevel);
	}

	// Znajduje ojca z taka sama osia podzialu i ktory ma tylko
	// jedno dziecko (ktore spelnia ten sam warunek)
	template <class ElementContainer>
	uint Tree<ElementContainer>::FindSimilarParent(vector<u32> &parents,uint nNode,uint axis) const {
		const Node &node=nodes[nNode];
		if(node.ClipLeft()>(&pMin.x)[node.Axis()]-5.0f&&node.ClipRight()<(&pMax.x)[node.Axis()]+5.0f) return ~0;
		if(axis==node.Axis()) return nNode;
		if(nNode==0) return ~0;
		return FindSimilarParent(parents,parents[nNode],axis);
	}

	template <class ElementContainer>
	void Tree<ElementContainer>::Build(vector<Index> &indices,vector<u32> &parents,uint nNode,
								Vec3f min,Vec3f max,uint level,bool sah) {
		maxLevel=Max(maxLevel,level+1);
		
/*		{ // eliminating duplicates
	RESORT:
			std::sort(indices.begin(),indices.end());
			for(int n=1;n<indices.size();n++) {
				if(indices[n].idx==indices[n-1].idx) {
					indices[n-1].min=VMin(indices[n-1].min,indices[n].min);
					indices[n-1].max=VMax(indices[n-1].max,indices[n].max);
					indices[n]=indices.back(); indices.pop_back();
					printf("x"); fflush(stdout);
					goto RESORT;
				}
			}
		} */

//		if(sizeoi(CElement)!=64) sah=0;
		/*{
			float sSize; { Vec3p s=pMax-pMin; sSize=s.x*(s.y+s.z)+s.y*s.z; }
			Vec3p size=max-min;
			float nodeSize=((size.x*(size.y+size.z)+size.y*size.z) / sSize);
			float density=float(indices.size())/nodeSize;
			nodes[nNode].density=density*0.5f;
		}*/
		if(level>Max(0,desiredMaxLevel-10)||sizeof(CElement)!=64) sah=0;

		float split; int axis;
		FindSplit(indices,min,max,axis,split);
		if(sah) if(!SAH(indices,min,max,axis,split)) sah=0;

		float leftMax=-1.0f/0.0f,rightMin=1.0f/0.0f;
		SplitIndices(elements,indices,axis,split,sah?0.0f:avgSize);

		int right=indices.size()-1;
		if(level>=desiredMaxLevel) {
			double sum=0;
			for(int n=0;n<indices.size();n++) {
				float min=(&indices[n].min.x)[axis];
				float max=(&indices[n].max.x)[axis];
				sum+=Lerp(min,max,0.5f);
			}
			sum/=double(indices.size());
			split=sum;
			right=indices.size()/2-1;
			leftMax=(&pMin.x)[axis];
			rightMin=(&pMax.x)[axis];
			for(int n=0;n<indices.size();n++) {
				float min=(&indices[n].min.x)[axis];
				float max=(&indices[n].max.x)[axis];
				if(n<=right) rightMin=Min(rightMin,min);
				else leftMax=Max(leftMax,max);
			}
		}
		else {
			right=indices.size()-1;
			for(int n=0;n<=right;n++) {
				float pos,min,max; {
					min=(&indices[n].min.x)[axis];
					max=(&indices[n].max.x)[axis];
					pos=Lerp(min,max,0.5f);
				}
				if(pos>=split) {
					Swap(indices[n--],indices[right--]);
					rightMin=Min(rightMin,min);
				}
				else leftMax=Max(leftMax,max);
			}
		}

		int numLeft=right+1;
		int numRight=indices.size()-numLeft;

		Vec3p maxL=max; (&maxL.x)[axis]=split;
		Vec3p minR=min; (&minR.x)[axis]=split;

		if((numLeft==0||numRight==0)) {
			uint sameAxisParent=FindSimilarParent(parents,parents[nNode],axis);
			if(sameAxisParent!=~0) {
				if(numLeft==0) Build(indices,parents,nNode,minR,max,level+1,1);
				if(numRight==0) Build(indices,parents,nNode,min,maxL,level+1,1);
				return;
			}
		}

		{
			Node &node=nodes[nNode];
			node.clip[0]=leftMax;
			node.clip[1]=rightMin;
			node.val[0]=axis<<30;
			node.val[1]=0;
		}

		if(numLeft) {
			bool leftLeaf=numLeft<=1; if(!leftLeaf) {
				int idx=indices[0].idx; leftLeaf=1;
				for(int n=1;n<=right;n++) if(indices[n].idx!=idx) { leftLeaf=0; break; }
			}
			if(leftLeaf) nodes[nNode].val[0]|=indices[0].idx|Node::leafMask;
			else {
				uint leftIdx=nodes.size();
				nodes[nNode].val[0]|=leftIdx;

				parents.push_back(nNode);
				nodes.push_back(Node());
				if(numRight) {
					vector<Index> inds(numLeft);
					std::copy(indices.begin(),indices.begin()+numLeft,inds.begin());
					Build(inds,parents,leftIdx,min,maxL,level+1,1);
				}
				else Build(indices,parents,leftIdx,min,maxL,level+1,1);
			}
		}
		if(numRight) {
			bool rightLeaf=numRight<=1; if(!rightLeaf) {
				int idx=indices[right+1].idx; rightLeaf=1;
				for(int n=right+2;n<indices.size();n++) if(indices[n].idx!=idx) { rightLeaf=0; break; }
			}
			if(rightLeaf) nodes[nNode].val[1]|=indices[right+1].idx|Node::leafMask;
			else {
				uint rightIdx=nodes.size();
				nodes[nNode].val[1]|=rightIdx;
				parents.push_back(nNode);
				nodes.push_back(Node());
				if(numLeft) {
					std::copy(indices.begin()+numLeft,indices.end(),indices.begin());
					indices.resize(numRight);
				}
				Build(indices,parents,rightIdx,minR,max,level+1,1);
			}
		}
	}

	template <class ElementContainer>
	void Tree<ElementContainer>::OptimizeBFS() {
		vector<uint> newPos(nodes.size()); {
			std::queue<int> inds;
			inds.push(0);
			int pos=0;

			while(!inds.empty()) {
				int ind=inds.front(); inds.pop();
				const Node &node=nodes[ind];
				newPos[ind]=pos++;

				if(!node.Child1IsLeaf()&&node.Child1()) inds.push(node.Child1());
				if(!node.Child2IsLeaf()&&node.Child2()) inds.push(node.Child2());
			}
		}

		vector<Node,AlignedAllocator<Node>> newNodes(nodes.size());

		for(int n=0;n<nodes.size();n++) {
			Node &node=newNodes[newPos[n]];
			node=nodes[n];
			if(!node.Child1IsLeaf())
				node.val[0]=newPos[node.Child1()]|(node.val[0]&~Node::idxMask);
			if(!node.Child2IsLeaf())
				node.val[1]=newPos[node.Child2()]|(node.val[1]&~Node::idxMask);
		}

		nodes.swap(newNodes);
	}

}

#include "tree_box.h"

template class bih::Tree<TriangleVector>;
template class bih::Tree<TreeBoxVector<bih::Tree<TriangleVector>>>;

