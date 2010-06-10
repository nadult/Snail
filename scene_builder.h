#ifndef RTRACER_SCENE_BUILDER_H
#define RTRACER_SCENE_BUILDER_H

#include "rtbase.h"
#include "tree_box.h"

template <class StaticTree>
class SceneBuilder {
public:
	struct Object {
		Matrix<Vec4f> preTrans;
		StaticTree *tree;
		BBox bBox;
	};
	
	struct Instance {
		Matrix<Vec4f> trans;
		u32 objId;
	};

	// box includes preTrans (it can be more optimized than just tree->BBox()*preTrans)
	void AddObject(StaticTree *tree,const Matrix<Vec4f> &preTrans,const BBox &box) {
		Object newObj;
		newObj.tree=tree;
		newObj.bBox=box;
		newObj.preTrans=preTrans;
		objects.push_back(newObj);
	}

	void AddInstance(int objId,const Matrix<Vec4f> &trans) {
		Instance newInst;
		newInst.trans=trans;
		newInst.objId=objId;
		instances.push_back(newInst);
	}

	typedef TreeBox<StaticTree> Elem;

	TreeBoxVector<StaticTree> ExtractElements() const {
		vector<Elem,AlignedAllocator<Elem> > elements;

		for(int n=0;n<instances.size();n++) {
			const Instance &inst=instances[n];
			const Object &obj=objects[inst.objId];
			Matrix<Vec4f> mat=obj.preTrans*inst.trans;
			BBox box=obj.bBox*inst.trans;
			elements.push_back(Elem(obj.tree,mat,box));
		}

		return TreeBoxVector<StaticTree>(elements);
	}
	
	vector<Object> objects;
	vector<Instance> instances;
};


#endif

