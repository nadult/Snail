#ifndef RTRACER_BIH_TREE_H
#define RTRACER_BIH_TREE_H

#include "ray_group.h"
#include "tree_stats.h"
#include "context.h"
#include "triangle.h"

namespace bih {

	// Liscie nie sa przechowywane w drzewie
	// Zamiast odnosnika do liscia jest od razu odnosnik do obiektu
	class Node {
	public:
		enum { leafMask=1<<29, idxMask=0x1fffffff };

		inline i32 Child1() const { return val[0]&0x1fffffff; }
		inline i32 Child2() const { return val[1]&0x1fffffff; }
		inline i32 Child(uint idx) const { return val[idx]&0x1fffffff; }

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

	void FindSplit(const vector<Index> &indices,const Vec3f &min,const Vec3f &max,int &outAxis,float &outSplit);
	bool SAH(const vector<Index> &indices,const Vec3f &min,const Vec3f &max,int &outAxis,float &outSplit);
	
	template <class Element>
	void SplitIndices(const vector<Element,AlignedAllocator<Element> > &elements,
						vector<Index> &inds,int axis,float pos,float maxSize) {
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

	void SplitIndices(const TriVector &tris,vector<Index> &inds,int axis,float pos,float maxSize);
//	void OptimizeIndices(vector<Index> &indices);

	template <class Element_,class ShElement_>
	class Tree: public RefCounter {
	public:
		typedef Element_ Element;
		typedef ShElement_ ShElement;

		typedef vector<Element,AlignedAllocator<Element> > ElementContainer;
		typedef vector<ShElement,AlignedAllocator<ShElement> > ShElementContainer;

		enum { elementIsComplex=Element::isComplex };
		enum { isctFlags=Element::isctFlags|isct::fObject|isct::fStats };
		enum { filterSigns=1 };
		enum { desiredMaxLevel=60 }; // Can be more, depends on the scene
		
		Tree() { }

		BBox GetBBox() const { return BBox(pMin,pMax); }

		Vec3f FlatNormals(u32 elementId,u32 subElementId) const {
			return elements[elementId].Nrm(subElementId);
		}
		Vec3f Barycentric(const Vec3f &orig,const Vec3f &dir,int elementId,int subElementId) const {
			return elements[elementId].Barycentric(orig,dir,subElementId);
		}
		Vec3q Barycentric(const Vec3q &orig,const Vec3q &dir,int elementId,int subElementId) const {
			return elements[elementId].Barycentric(orig,dir,subElementId);
		}
		const typename Element::BaseElement &GetElement(int elem,int subElem) const {
			return elements[elem].GetElement(subElem);
		}

	private:
		struct GetShSimple  { INLINE static const ShElement &Get(const Tree *ptr,int elem,int subElem)
			{ return ptr->shElements[elem]; } };
		struct GetShComplex { INLINE static const ShElement &Get(const Tree *ptr,int elem,int subElem)
			{ return ptr->elements[elem].GetShElement(subElem); } };

	public:
		const ShElement &GetShElement(int elem,int subElem) const {
			return TSwitch<GetShComplex,GetShSimple,elementIsComplex>::Result::Get(this,elem,subElem);
		}

		Tree(const ElementContainer &elements,const ShElementContainer &shElements=ShElementContainer());

		void PrintInfo() const;
		uint FindSimilarParent(vector<u32> &parents,uint nNode,uint axis) const;
		void Build(vector<Index> &indices,vector<u32> &parents,uint nNode,const Vec3f &min,const Vec3f &max,uint level,bool);

	public:	
		template <int flags,template <int> class Selector>
		Isct<f32x4,1,isctFlags|flags>
			TraversePacket(const RayGroup<1,flags> &rays,const Selector<1> &selector) const
		{
			static_assert(flags&isct::fInvDir,"");

			int bitMask=selector[0];
			const Vec3q &dir=rays.Dir(0);
			Isct<f32x4,1,isctFlags|flags> out;

			bool split=1;

			if(Selector<1>::full||bitMask==0x0f) {	
				if(!filterSigns||GetVecSign(rays.Dir(0))!=8) {
					out=TraversePacket0(rays);
					split=0;
				}
			}

			if(split) {
				if(bitMask==0) return out;

				const Vec3q &orig=rays.Origin(0);
				for(int q=0;q<4;q++) {	
					int o=flags&isct::fShOrig?0:q;
					if(!(bitMask&(1<<q))) continue;
					f32x4 maxDist=rays.MaxDist(0);
					float tMaxDist=maxDist[q];

					Isct<float,1,isctFlags|flags> tOut=	
						TraverseMono<flags>
						(Vec3p(orig.x[o],orig.y[o],orig.z[o]),Vec3p(dir.x[q],dir.y[q],dir.z[q]),tMaxDist);
					out.Insert(tOut,q);
				}
			}

			return out;
		}

		template <int flags>
		INLINE Isct<f32x4,1,isctFlags|flags>
			TraversePacket(const RayGroup<1,flags> &rays) const {
			return TraversePacket(rays,FullSelector<1>());
		}

		template <int flags,int packetSize,template <int> class Selector>
		Isct<f32x4,packetSize,isctFlags|flags>
			TraversePacket(const RayGroup<packetSize,flags> &rays,const Selector<packetSize> &selector) const
		{
			static_assert(flags&isct::fInvDir,"");

			Isct<f32x4,packetSize,isctFlags|flags> out;	
			bool split=1;

			enum { reflected=!(flags&(isct::fPrimary|isct::fShadow)) };

			bool selectorsFiltered=packetSize<=(reflected?4:16);
			if(!Selector<packetSize>::full)
				for(int n=0;n<packetSize/4;n++)
					if(selector.Mask4(n)!=0x0f0f0f0f) {
						selectorsFiltered=0;
						break;
					}

			if((Selector<packetSize>::full||selectorsFiltered)&&packetSize<=(reflected?4:16)) {
				const Vec3q &dir=rays.Dir(0);
				bool signsFiltered=1;
				int msk=_mm_movemask_ps(_mm_shuffle_ps(_mm_shuffle_ps(dir.x.m,dir.y.m,0),dir.z.m,0+(2<<2)))&7;

				if(filterSigns) {					
					for(int n=0;n<packetSize;n++) if(GetVecSign(rays.Dir(n))!=msk) {
						signsFiltered=0;
						break;
					}
				}

				if(signsFiltered) {
					bool primary=flags&(isct::fPrimary|isct::fShadow)&&gVals[1];

					if(flags&isct::fShadow) {
						floatq dot=1.0f;
						for(int q=1;q<packetSize;q++) dot=Min(dot,rays.Dir(0)|rays.Dir(q));
						if(ForAny(dot<0.9998f)) primary=0;
					}
					out=primary?TraversePrimary(rays):TraversePacket0(rays);

					if(primary&&(flags&isct::fShadow)) out.Stats().Skip();
					split=0;
				}
			}

			if(split) {
				RayGroup<packetSize/4,flags> subGroups[4];
				rays.Split(subGroups);
				subGroups[0].lastShadowTri=rays.lastShadowTri;

				for(int q=0;q<4;q++) {
					Isct<f32x4,packetSize/4,isctFlags|flags> tOut=
						TraversePacket(subGroups[q],selector.SubSelector(q));
					if(q!=3) subGroups[q+1].lastShadowTri=tOut.LastShadowTri();
					out.Insert(tOut,q);
				}

			}

			return out;
		}

		template <int flags,int packetSize> INLINE Isct<f32x4,packetSize,isctFlags|flags>
			TraversePacket(const RayGroup<packetSize,flags> &rays) const {
			return TraversePacket(rays,FullSelector<packetSize>());
		}
	
		#include "bih/traverse_mono.h"
		#include "bih/traverse_packet.h"	
		#include "bih/traverse_primary.h"

		float avgSize;
		Vec3f pMin,pMax;
		vector<Node> nodes;
		ElementContainer elements;
		ShElementContainer shElements;

		int objectId,maxLevel;
		float maxDensity;
	};

}

#include "bih/tree_impl.h"


#endif


