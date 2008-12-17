#ifndef RTRACER_TREE_BOX_H
#define RTRACER_TREE_BOX_H

#include "ray_group.h"
#include "tree_stats.h"
#include "context.h"
#include "triangle.h"

	template <class BaseTree> class TreeBoxVector;

	template <class BaseTree_>
	class TreeBox {
	public:
		typedef BaseTree_ BaseTree;

		enum { isComplex=1 }; // contains hierarchy
		enum { isctFlags=BaseTree::isctFlags|isct::fElement };

		TreeBox() :tree(0) { }
		TreeBox(const BaseTree *tr,Matrix<Vec4f> m,const BBox &b)
			:trans(m),invTrans(Inverse(m)),bBox(b),tree(tr) { }

		Vec3f Nrm(int subElementId) const {
			return trans&tree->FlatNormals(subElementId,0);
		}

		Vec3f BoundMin() const { return bBox.min; }
		Vec3f BoundMax() const { return bBox.max; }
		BBox GetBBox() const { return bBox; }

		template <int flags>
		Isct<float,1,isctFlags|flags> Collide(const Vec3f &rOrig,const Vec3f &rDir,float maxDist) const {
			assert(tree);

			Isct<float,1,isctFlags|flags> out=
				tree->template TraverseMono<flags>(invTrans*rOrig,invTrans&rDir,maxDist);
			if(!(flags&isct::fShadow)) out.Element(0)=out.Object(0);

			return out;
		}

		template <int flags,int packetSize>
		Isct<f32x4,packetSize,isctFlags|flags>
			Collide(const RayGroup<packetSize,flags> &rays) const {
			assert(tree);

			Isct<f32x4,packetSize,isctFlags|flags> out;
			bool test=1;

		//	if(gVals[2]) {
			//	if(sharedOrigin) test=bBox.TestIP<packetSize>(rays.Origin(0),precompInv?rays.idir:tinv,maxDist);
			//	else
	//				for(int q=0;q<packetSize&&!test;q++)
	//				test|=bBox.TestI(rays.Origin(q),rays.IDir(q),f32x4(1.0f/0.0f));
		//	} else test=1;

			if(test) {
				RayGroup<packetSize,flags> trays(rays);
				trays *= invTrans;

				Isct<f32x4,packetSize,BaseTree::isctFlags|flags> tOut=tree->TraversePacket(trays);

				for(int q=0;q<packetSize;q++) out.Distance(q)=tOut.Distance(q);

				if(!(flags&isct::fShadow)) {
					for(int q=0;q<packetSize;q++) out.Element(q)=tOut.Object(q);
				//	for(int q=0;q<packetSize;q++) out.object[q]=0;
				}
			}
			else {
				for(int q=0;q<packetSize;q++)
					out.Distance(q)=1.0f/0.0f;
			}

			return out;
		}
		void Serialize(const Serializer &sr) {
			ThrowException("TODO: TreeBox Serializer not avaliable");
		}

//	private:
		friend class TreeBoxVector<BaseTree>;
		Matrix<Vec4f> trans,invTrans;
		BBox bBox;
		const BaseTree_ *tree;
	};


	template <class BaseTree_>
	class TreeBoxVector {
	public:
		typedef BaseTree_ BaseTree;

		typedef TreeBox<BaseTree> CElement;
		typedef typename BaseTree::SElement SElement;

		TreeBoxVector() { }
		TreeBoxVector(const vector<CElement,AlignedAllocator<CElement> > &e) :elems(e) { }

		void Serialize(Serializer &sr) { sr&elems; }

		INLINE const CElement &operator[](int elem) const { return elems[elem]; }
		INLINE const CElement &GetCElement(int elem) const { return elems[elem]; }

		INLINE const SElement GetSElement(int obj,int subElem) const {
			const CElement &elem=elems[obj];
			SElement out=elem.tree->GetSElement(subElem,0);
			out *= elem.trans;
			return out;
		}
		size_t size() const { return elems.size(); }
		int mem_size() const { /*TODO*/ return elems.size(); }

		Vec3f BoundMin(int n) const { return elems[n].BoundMin(); }
		Vec3f BoundMax(int n) const { return elems[n].BoundMax(); }

		vector<CElement,AlignedAllocator<CElement> > elems;
	};


#endif


