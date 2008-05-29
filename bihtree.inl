void GenBIHIndices(const vector<Triangle> &tris,vector<BIHIdx> &out,float maxSize,uint maxSplits);

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
	nodes.push_back(BIHNode(0));
	
	double avgSize=sumSize.x+sumSize.y+sumSize.z;
	avgSize/=3.0*objects.size();

	vector<BIHIdx> indices;
	GenBIHIndices(obj,indices,avgSize*1.75f,16*objects.size());

	Build(indices,0,0,indices.size()-1,pMin,pMax,0);
	printf("Indices: %d Avg size: %.2f\n",indices.size(),avgSize);

	for(uint n=0;n<nodes.size();n++) {
		BIHNode &node=nodes[n];

		uint axis=node.Axis();
		node.flags=node.ClipLeft()>(&pMin.x)[axis]-5.0f&&node.ClipRight()<(&pMax.x)[axis]+5.0f?0:1;
	}
}

// Znajduje ojca z taka sama osia podzialu i ktory ma tylko
// jedno dziecko (ktore spelnia ten sam warunek)
template <class Object>
uint BIHTree<Object>::FindSimilarParent(uint nNode,uint axis) const {
	const BIHNode &node=nodes[nNode];

	if(node.ClipLeft()>(&pMin.x)[node.Axis()]-5.0f&&node.ClipRight()<(&pMax.x)[node.Axis()]+5.0f)
		return ~0;
	if(axis==node.Axis())
		return nNode;

	if(nNode==0) return ~0;
	FindSimilarParent(node.parent,axis);
}

template <class Object>
void BIHTree<Object>::PrintInfo(uint nNode,uint level) const {
	if(nNode==0) {
		printf("Objects:%8d * %2d = %6.2fMB\n",objects.size(),sizeof(Object),double(objects.size()*sizeof(Object))*0.000001);
		printf("Nodes:  %8d * %2d = %6.2fMB\n\n",nodes.size(),sizeof(BIHNode),double(nodes.size()*sizeof(BIHNode))*0.000001);
		return;

		double objSize=0;
		for(int n=0;n<objects.size();n++) {
			const Object &o=objects[n];
			Vec3p bMin=o.BoundMin(),bMax=o.BoundMax();
			objSize+=bMax.x+bMax.y+bMax.z-bMin.x-bMin.y-bMin.z;
		//	printf("%d box: (%f %f %f) (%f %f %f)\n",n,bMin.x,bMin.y,bMin.z,bMax.x,bMax.y,bMax.z);
		}

	//	printf("Box: (%f %f %f) (%f %f %f)\n",pMin.x,pMin.y,pMin.z,pMax.x,pMax.y,pMax.z);
	//	printf("Average object size: %.2f\n",objSize/double(6*objects.size()));
	}
	for(int n=0;n<level;n++) printf(" ");
	const BIHNode &node=nodes[nNode];
	if(node.Axis()==3) { printf("Leaf %d pointing to %d\n",node.Object()); }
	else {
		printf("Node: %d %f %f\n",node.Axis(),node.ClipLeft(),node.ClipRight());
		PrintInfo(node.Child()+0,level+1);
		PrintInfo(node.Child()+1,level+1);
	}
}

inline uint MaxAxis(Vec3p size) {
	uint axis=0;
	float s=size.x;
	if(size.y>s) { s=size.y; axis=1; }
	if(size.z>s) axis=2;
	return axis;
}

template <class Object>
void BIHTree<Object>::Build(vector<BIHIdx> &indices,uint nNode,int first,int last,Vec3p min,Vec3p max,uint level) {
	int count=last-first+1;

	if(count<=1) {
		if(count==0) throw Exception("Error while building BIHTree: there should be no empty leafs!");
		nodes[nNode].SetLeaf(indices[first].idx);
		return;
	}

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
		sum/=double(count);
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
		right=first+count/2-1;
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
		uint sameAxisParent=FindSimilarParent(nodes[nNode].parent,axis);
		if(sameAxisParent!=~0) {
			if(numLeft==0) Build(indices,nNode,first,last,minR,max,level+1);
			if(numRight==0) Build(indices,nNode,first,last,min,maxL,level+1);
			return;
		}
	}

	{
		BIHNode &node=nodes[nNode];
		if(numLeft==0) leftMax=(&pMin.x)[axis]-10.0f;
		if(numRight==0) rightMin=(&pMax.x)[axis]+10.0f;

		node.SetNode(axis,leftMax,rightMin,nodes.size());
	}

	uint cLeft,cRight;
	if(numLeft) {
		cLeft=nodes.size();
		nodes.push_back(BIHNode(nNode));
	}
	if(numRight) {
		cRight=nodes.size();
		nodes.push_back(BIHNode(nNode));
	}

	if(numLeft) Build(indices,cLeft,first,right,min,maxL,level+1);
	if(numRight) Build(indices,cRight,right+1,last,minR,max,level+1);
}


