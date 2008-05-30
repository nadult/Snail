void GenBIHIndices(const vector<Triangle> &tris,vector<BIHIdx> &out,float maxSize,uint maxSplits);

namespace {

	inline uint MaxAxis(Vec3p size) {
		uint axis=0;
		float s=size.x;
		if(size.y>s) { s=size.y; axis=1; }
		if(size.z>s) axis=2;
		return axis;
	}

}

template <class Object>
BIHTree<Object>::BIHTree(const vector<Object> &obj) :objects(obj) {
	if(!objects.size()) return;

	pMin=objects[0].BoundMin();
	pMax=objects[0].BoundMax();

	Vec3p sumSize(0,0,0);
	for(uint n=1;n<objects.size();n++) {
		Vec3p min=objects[n].BoundMin(),max=objects[n].BoundMax();
		sumSize+=max-min;

		pMin=VMin(pMin,min);
		pMax=VMax(pMax,max);
	}
	nodes.push_back(BIHNode());
	
	double avgSize=sumSize.x+sumSize.y+sumSize.z;
	avgSize/=3.0*objects.size();

	vector<BIHIdx> indices;
	GenBIHIndices(obj,indices,avgSize*1.75f,32*objects.size());
	printf("Indices: %d Avg size: %.2f\n",indices.size(),avgSize);

	vLeafs=0;
	vector<u32> parents; parents.push_back(0);
	Build(indices,parents,0,0,indices.size()-1,pMin,pMax,0);
}

template <class Object>
void BIHTree<Object>::PrintInfo() const {
	printf("Objects:%8d * %2d = %6.2fMB\n",objects.size(),sizeof(Object),double(objects.size()*sizeof(Object))*0.000001);
	printf("Nodes:  %8d * %2d = %6.2fMB\n",nodes.size(),sizeof(BIHNode),double(nodes.size()*sizeof(BIHNode))*0.000001);
	printf("vleafs: %8d * %2d = %6.2fMB (they are not stored)\n\n",vLeafs,sizeof(BIHNode),double(vLeafs*sizeof(BIHNode))*0.000001);
}

// Znajduje ojca z taka sama osia podzialu i ktory ma tylko
// jedno dziecko (ktore spelnia ten sam warunek)
template <class Object>
uint BIHTree<Object>::FindSimilarParent(vector<u32> &parents,uint nNode,uint axis) const {
	const BIHNode &node=nodes[nNode];
	if(node.ClipLeft()>(&pMin.x)[node.Axis()]-5.0f&&node.ClipRight()<(&pMax.x)[node.Axis()]+5.0f) return ~0;
	if(axis==node.Axis()) return nNode;
	if(nNode==0) return ~0;
	FindSimilarParent(parents,parents[nNode],axis);
}

template <class Object>
void BIHTree<Object>::Build(vector<BIHIdx> &indices,vector<u32> &parents,uint nNode,int first,int last,Vec3p min,Vec3p max,uint level) {
	uint axis=MaxAxis(max-min);
	
	float split=Lerp((&min.x)[axis],(&max.x)[axis],0.5f);
	float leftMax=(&pMin.x)[axis],rightMin=(&pMax.x)[axis];
	int right=last;

	if(level>=maxLevel) { // Od teraz dzielimy obiekty rowno po polowie
		double sum=0;
		for(int n=first;n<=last;n++) {
			float min=(&indices[n].min.x)[axis];
			float max=(&indices[n].max.x)[axis];
			sum+=Lerp(min,max,0.5f);
		}
		sum/=double(last-first+1);
		split=sum;
	}

	for(int n=first;n<=right;n++) {
		float pos,min,max; {
			min=(&indices[n].min.x)[axis];
			max=(&indices[n].max.x)[axis];
			pos=Lerp(min,max,0.5f);
		}
		if(pos>=split) {
			Swap(indices[n--],indices[right--]);
			rightMin=Min(rightMin,min);
		}
		else {
			leftMax=Max(leftMax,max);
		}
	}

	if(level>=maxLevel) {
		right=first+(last-first+1)/2-1;
		leftMax=(&pMin.x)[axis];
		rightMin=(&pMax.x)[axis];
		for(int n=first;n<=last;n++) {
			float min=(&indices[n].min.x)[axis];
			float max=(&indices[n].max.x)[axis];
			if(n<=right) rightMin=Min(rightMin,min);
			else leftMax=Max(leftMax,max);
		}
	}

	int numLeft=right-first+1;	
	int numRight=last-right;

	Vec3p maxL=max; (&maxL.x)[axis]=split;
	Vec3p minR=min; (&minR.x)[axis]=split;

	if((numLeft==0||numRight==0)) {
		uint sameAxisParent=FindSimilarParent(parents,parents[nNode],axis);
		if(sameAxisParent!=~0) {
			if(numLeft==0) Build(indices,parents,nNode,first,last,minR,max,level+1);
			if(numRight==0) Build(indices,parents,nNode,first,last,min,maxL,level+1);
			return;
		}
	}

	{
		BIHNode &node=nodes[nNode];
		if(numLeft==0) leftMax=(&pMin.x)[axis]-10.0f;
		if(numRight==0) rightMin=(&pMax.x)[axis]+10.0f;
		node.clip[0]=leftMax;
		node.clip[1]=rightMin;
		node.val[0]=axis<<30;
		node.val[1]=0;
	}

	uint cLeft,cRight;
	bool leftLeaf=0,rightLeaf=0;

	if(numLeft) {
		nodes[nNode].val[1]|=(1<<30);
		cLeft=nodes.size();
		leftLeaf=numLeft<=1; if(!leftLeaf) {
			int idx=indices[first].idx; leftLeaf=1;
			for(int n=first+1;n<=right;n++) if(indices[n].idx!=idx) { leftLeaf=0; break; }
		}
		if(leftLeaf) {
			nodes[nNode].val[0]|=indices[first].idx|BIHNode::leafMask;
			vLeafs++;
		}
		else {
			nodes[nNode].val[0]|=nodes.size();
			parents.push_back(nNode);
			nodes.push_back(BIHNode());
		}
	}
	if(numRight) {
		nodes[nNode].val[1]|=(1<<31);
		cRight=nodes.size();
		rightLeaf=numRight<=1; if(!rightLeaf) {
			int idx=indices[right+1].idx; rightLeaf=1;
			for(int n=right+2;n<=last;n++) if(indices[n].idx!=idx) { rightLeaf=0; break; }
		}
		if(rightLeaf) {
			nodes[nNode].val[1]|=indices[right+1].idx|BIHNode::leafMask;
			vLeafs++;
		}
		else {
			nodes[nNode].val[1]|=nodes.size();
			parents.push_back(nNode);
			nodes.push_back(BIHNode());
		}
	}

	if(!leftLeaf&&numLeft)  Build(indices,parents,cLeft,first,right,min,maxL,level+1);
	if(!rightLeaf&&numRight) Build(indices,parents,cRight,right+1,last,minR,max,level+1);
}


