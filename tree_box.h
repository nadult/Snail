#ifndef RTRACER_TREE_BOX_H
#define RTRACER_TREE_BOX_H

#include "ray_group.h"
#include "tree_stats.h"
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
			:trans(m),invTrans(Inverse(m)),bBox(b),tree(tr) { Update(); }

		Vec3f Nrm(int subElementId) const {
			return trans&tree->FlatNormals(subElementId,0);
		}

		void Update() {
			noTrans=trans.x==Vec4f(1.0f,0.0f,0.0f,0.0f)&&
					trans.y==Vec4f(0.0f,1.0f,0.0f,0.0f)&&
					trans.z==Vec4f(0.0f,0.0f,1.0f,0.0f)&&
					trans.w==Vec4f(0.0f,0.0f,0.0f,1.0f);
		}

		Vec3f BoundMin() const { return bBox.min; }
		Vec3f BoundMax() const { return bBox.max; }
		BBox GetBBox() const { return bBox; }

		template <int flags>
		void Collide(FContext<flags> &c,int idx) const {
			assert(tree);
			Vec3f tOrig=invTrans*c.origin;
			Vec3f tDir =invTrans&c.dir;
			Vec3f tIDir=SafeInv(tDir);
			int obj=~0,elem;

			FContext<flags> tc(c);
			tc.origin=tOrig;
			tc.dir=tDir;
			tc.iDir=tIDir;
			tc.object=&obj;
			tc.element=&elem;

			tree->TraverseMono(tc);
			if(!(flags&isct::fShadow)) {
				if(obj!=~0) {
					c.object[0]=idx;
					c.element[0]=obj;
				}
			}
		}

		template <int flags,int size>
		bool Collide(Context<size,flags> &c,int idx) const {
			assert(tree);

			bool test=0;

			if(noTrans) test=1;
			else {
				if(flags&isct::fShOrig)
					test=bBox.TestIP<size>(c.Origin(0),c.rays.IDirPtr(),c.distance);
				else for(int q=0;q<size&&!test;q++)
					test|=bBox.TestI(c.Origin(q),c.IDir(q),c.distance[q]);
			}

			if(test) {
				Vec3q tDir[size],idir[size],tOrig[flags&isct::fShOrig?1:size];
				if(!noTrans) {
					for(int q=0;q<size;q++) {
						tDir[q]=invTrans&c.Dir(q);
						idir[q]=SafeInv(tDir[q]);
					}
					for(int q=0;q<sizeof(tOrig)/sizeof(Vec3q);q++) tOrig[q]=invTrans*c.Origin(q);
				}

				i32x4 object[size],temp[size];
				for(int n=0;n<size;n++) object[n]=i32x4(~0);

				TreeStats<1> stats;

				Context<size,flags> tc(noTrans?c.rays.OriginPtr():tOrig,noTrans?c.rays.DirPtr():tDir,
										noTrans?c.rays.IDirPtr():idir,c.distance,object,temp,&stats);
				tree->TraversePacket(tc);
				c.UpdateStats(stats);

				if(!(flags&isct::fShadow)) {
					for(int q=0;q<size;q++) {
						i32x4b mask=object[q]!=~0;
						c.element[q]=Condition(mask,object[q],c.element[q]);
						c.object[q]=Condition(mask,i32x4(idx),c.object[q]);
					}
				}
			}
			
			return 0;
		}
		void Serialize(const Serializer &sr) {
			ThrowException("TODO: TreeBox Serializer not avaliable");
		}

//	private:
		friend class TreeBoxVector<BaseTree>;
		Matrix<Vec4f> trans,invTrans;
		BBox bBox;
		const BaseTree_ *tree;
		bool noTrans;
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
		const BBox &GetBBox(int n) const { return elems[n].bBox; }

		vector<CElement,AlignedAllocator<CElement> > elems;
	};


#endif


