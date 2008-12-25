#ifndef RTRACER_BIH_TREE_H
#define RTRACER_BIH_TREE_H

#include "ray_group.h"
#include "tree_stats.h"
#include "triangle.h"


namespace bih {

	// Liscie nie sa przechowywane w drzewie
	// Zamiast odnosnika do liscia jest od razu odnosnik do obiektu
	class Node {
	public:
		enum { leafMask=1<<29, idxMask=0x1fffffff };

		inline i32 Child1() const { return val[0]&idxMask; }
		inline i32 Child2() const { return val[1]&idxMask; }
		inline i32 Child(uint idx) const { return val[idx]&idxMask; }

		inline bool Child1IsLeaf() const { return val[0]&leafMask; }
		inline bool Child2IsLeaf() const { return val[1]&leafMask; }
		inline bool ChildIsLeaf(uint idx) const { return val[idx]&leafMask; }

		inline int Axis() const { return (val[0]>>30); }
		inline float ClipLeft() const { return clip[0]; }
		inline float ClipRight() const { return clip[1]; }

		inline bool DenseNode() const { return val[1]&(1<<30); }

		float clip[2];
		u32 val[2];
	};

}

namespace baselib { template<> struct SerializeAsPOD<bih::Node> { enum { value=1 }; }; }

namespace bih {

	class Index {
	public:
		Index() { }
		Index(int i,const Vec3f &mi,const Vec3f &ma,float mul) :min(mi),max(ma),idx(i) {
			Vec3f vsize=max-min;
			size=Max(vsize.x,Max(vsize.y,vsize.z))*mul;
		}
		bool operator<(const Index &i) const { return idx<i.idx; }

		Vec3f min,max;
		float size;
		i32 idx;
	};

//	void GenBIHIndices(const TriVector &tris,vector<Index> &out,float maxSize,uint maxSplits);

	void FindSplit(const vector<Index> &indices,const BBox &box,int &outAxis,float &outSplit);
	bool SAH(const vector<Index> &indices,const BBox &box,int &outAxis,float &outSplit);
	
	template <class ElemContainer>
	void SplitIndices(const ElemContainer &elements,vector<Index> &inds,int axis,float pos,float maxSize) {
		typedef typename ElemContainer::CElement Element;
	//	if(inds.size()<=(maxSize>0.0f?16:64)) return;
		if(inds.size()<=2) return;
		maxSize*=1.5f;

		for(int n=0,end=inds.size();n<end;n++) {
			Index &idx=inds[n];
			if(idx.size<maxSize) continue;
			if((&idx.min.x)[axis]>=pos||(&idx.max.x)[axis]<=pos) continue;

			const Element &elem=elements[idx.idx];

			Vec3f min1=idx.min,min2=idx.min,max1=idx.max,max2=idx.max;
			(&max1.x)[axis]=(&min2.x)[axis]=pos;

			idx=Index(idx.idx,min1,max1,1.0f);
			inds.push_back(Index(idx.idx,min2,max2,1.0f));
		}
	}

	void SplitIndices(const TriangleVector &tris,vector<Index> &inds,int axis,float pos,float maxSize);
//	void OptimizeIndices(vector<Index> &indices);

	template <class ElementContainer_>
	class Tree: public RefCounter {
	public:
		typedef ElementContainer_ ElementContainer;
		typedef typename ElementContainer::CElement CElement;
		typedef typename ElementContainer::SElement SElement;

		enum { isComplex=CElement::isComplex };
		enum { isctFlags=CElement::isctFlags|isct::fObject|isct::fStats };
		enum { filterSigns=1 };
		enum { desiredMaxLevel=60 }; // Can be more, depends on the scene
		
		Tree() { }

		void Serialize(Serializer&);

		BBox GetBBox() const { return bBox; }

	//	Vec3f FlatNormals(u32 elementId,u32 subElementId) const {
	//		return elements[elementId].Nrm(subElementId);
	//	}

		const SElement GetSElement(int elem,int subElem) const { return elements.GetSElement(elem,subElem); }

		Tree(const ElementContainer &elements);
		void PrintInfo() const;

		void Construct(const ElementContainer &elements,bool fast=0);

	private:
		uint FindSimilarParent(vector<u32> &parents,uint nNode,uint axis) const;
		void Build(vector<Index> &indices,vector<u32> &parents,uint nNode,BBox box,uint level,bool,bool);
		void OptimizeBFS();
	
		template <int flags,int size> void TraversePacket0(Context<size,flags>&) const;
		template <int flags,int size> void TraversePrimary(Context<size,flags>&) const;

	public:	
		template <int flags> void TraverseMono(FContext<flags>&) const;

		template <int flags,template <int> class Selector> void
			TraversePacket(Context<1,flags>&,const Selector<1>&) const;

		template <int flags,int size,template <int> class Selector> void
			TraversePacket(Context<size,flags>&,const Selector<size>&) const;

		template <int flags>
		INLINE void TraversePacket(Context<1,flags> &c) const {
			TraversePacket(c,FullSelector<1>());
		}

		template <int flags,int size> INLINE void TraversePacket(Context<size,flags> &c) const {
			TraversePacket(c,FullSelector<size>());
		}

		vector<Node,AlignedAllocator<Node> > nodes;
		ElementContainer elements;

		BBox bBox;
		float avgSize;
		int maxLevel;
	};


}

#endif


