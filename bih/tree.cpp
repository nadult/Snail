#include <algorithm>
#include <stdio.h>
#include "bih/tree.h"
#include <iostream>
#include <ostream>

namespace bih {

	namespace {

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

}
