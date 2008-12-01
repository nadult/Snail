#ifndef RTRACER_BIH_TREE_IMPL_H
#define RTRACER_BIH_TREE_IMPL_H

#include <algorithm>
#include <stdio.h>
#include <iostream>
#include <ostream>
#include "bih/tree.h"


namespace bih {

	namespace {

		inline uint MaxAxis(Vec3p size) {
			uint axis=0;
			float s=size.x;
			if(size.y>s) { s=size.y; axis=1; }
			if(size.z>s) axis=2;
			return axis;
		}

	}

	template <class Element,class ShElement>
	Tree<Element,ShElement>::Tree(const ElementContainer &objs,const ShElementContainer &shElems) {

		elements.resize(objs.size());
		shElements=shElems;

		if(!elements.size()) {
			nodes.push_back(Node());
			nodes[0].clip[0]=-1.0f/0.0f;
			nodes[0].clip[1]=1.0f/0.0f;
			nodes[0].val[0]=nodes[0].val[1]=0;
			return;
		}
	/*	for(int n=0;n<objs.size();n++) {
			elements[n]=objs[n];
			Vec3p nrm=elements[n].Nrm();
			elements[n].SetFlag2((nrm.x<0?1:0)+(nrm.y<0?1:0)+(nrm.z<0?1:0));
		} */
		std::copy(objs.begin(),objs.end(),elements.begin());

		pMin=elements[0].BoundMin();
		pMax=elements[0].BoundMax();
		maxLevel=0;

		Vec3p sumSize(0,0,0);
		for(uint n=1;n<elements.size();n++) {
			const Element &elem=elements[n];
			Vec3p min=elem.BoundMin(),max=elem.BoundMax();
			sumSize+=max-min;

			pMin=VMin(pMin,min);
			pMax=VMax(pMax,max);
		}
		nodes.push_back(Node());
		
		avgSize=sumSize.x+sumSize.y+sumSize.z;
		avgSize/=3.0*elements.size();

	//	printf("."); fflush(stdout);
		vector<Index> indices; indices.reserve(elements.size()*16);

		for(int n=0;n<elements.size();n++) {
			const Element &elem=elements[n];
			indices.push_back(Index(n,elem.BoundMin(),elem.BoundMax(),1.0f));
		}

		vector<u32> parents; parents.push_back(0);

		Build(indices,parents,0,pMin,pMax,0,1);
	}

	template <class Element,class ShElement>
	void Tree<Element,ShElement>::PrintInfo() const {
		double nodeBytes=nodes.size()*sizeof(Node);
		double objBytes=elements.size()*sizeof(Element);
		double shBytes=shElements.size()*sizeof(ShElement);

		printf("Elems:  %8d * %2d = %6.2fMB\n",elements.size(),sizeof(Element),objBytes*0.000001);
		printf("ShElems:%8d * %2d = %6.2fMB\n",shElements.size(),sizeof(ShElement),shBytes*0.000001);
		printf("Nodes: %8d * %2d = %6.2fMB\n",nodes.size(),sizeof(Node),nodeBytes*0.000001);
		printf("~ %.0f bytes per triangle\n",(nodeBytes+objBytes+shBytes)/double(elements.size()));
		printf("Levels: %d\n\n",maxLevel);
	}

	// Znajduje ojca z taka sama osia podzialu i ktory ma tylko
	// jedno dziecko (ktore spelnia ten sam warunek)
	template <class Element,class ShElement>
	uint Tree<Element,ShElement>::FindSimilarParent(vector<u32> &parents,uint nNode,uint axis) const {
		const Node &node=nodes[nNode];
		if(node.ClipLeft()>(&pMin.x)[node.Axis()]-5.0f&&node.ClipRight()<(&pMax.x)[node.Axis()]+5.0f) return ~0;
		if(axis==node.Axis()) return nNode;
		if(nNode==0) return ~0;
		return FindSimilarParent(parents,parents[nNode],axis);
	}

	template <class Element,class ShElement>
	void Tree<Element,ShElement>::Build(vector<Index> &indices,vector<u32> &parents,uint nNode,
								const Vec3f &min,const Vec3f &max,uint level,bool sah) {
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

//		if(sizeof(Element)!=64) sah=0;
		/*{
			float sSize; { Vec3p s=pMax-pMin; sSize=s.x*(s.y+s.z)+s.y*s.z; }
			Vec3p size=max-min;
			float nodeSize=((size.x*(size.y+size.z)+size.y*size.z) / sSize);
			float density=float(indices.size())/nodeSize;
			nodes[nNode].density=density*0.5f;
		}*/
		if(level>Max(0,desiredMaxLevel-10)||sizeof(Element)!=64) sah=0;

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

		if((numLeft==0||numRight==0)&&!sah) {
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
			nodes[nNode].val[1]=0;
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

}

#endif

