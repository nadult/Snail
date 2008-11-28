#ifndef RTRACER_TREE_BOX_H
#define RTRACER_TREE_BOX_H

#include "ray_group.h"
#include "tree_stats.h"
#include "context.h"
#include "triangle.h"

	template <class BaseTree_>
	class TreeBox {
	public:
		typedef BaseTree_ BaseTree;
		typedef typename BaseTree::Element BaseElement;
		typedef typename BaseTree::ShElement ShElement;

		enum { isComplex=1 }; // contains hierarchy
		enum { isctFlags=BaseTree::isctFlags|isct::fElement };

		TreeBox() :tree(0) { }
		TreeBox(const BaseTree *tr,Matrix<Vec4f> m,const BBox &b)
			:trans(m),invTrans(Inverse(m)),bBox(b),tree(tr) { }

		Vec3f Nrm(int subElementId) const {
			return trans&tree->FlatNormals(subElementId,0);
		}
		Vec3f Barycentric(const Vec3f &orig,const Vec3f &dir,int subElementId) const {
			const Vec3f tDir=invTrans&dir;
			const Vec3f tOrig=invTrans*orig;

			return tree->Barycentric(tOrig,tDir,subElementId,0);
		}
		Vec3q Barycentric(const Vec3q &orig,const Vec3q &dir,int subElementId) const {
			const Vec3q tDir=invTrans&dir;
			const Vec3q tOrig=invTrans*orig;

			return tree->Barycentric(tOrig,tDir,subElementId,0);
		}
		const typename BaseTree::Element &GetElement(int elem) const {
			return tree->GetElement(elem,0);
		}

		const ShElement &GetShElement(int id) const { return tree->GetShElement(id,0); }

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
			bool test=0;

		//	if(gVals[2]) {
			//	if(sharedOrigin) test=bBox.TestIP<packetSize>(rays.Origin(0),precompInv?rays.idir:tinv,maxDist);
			//	else
					for(int q=0;q<packetSize&&!test;q++)
					test|=bBox.TestI(rays.Origin(q),rays.IDir(q),f32x4(1.0f/0.0f));
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

	private:
		Matrix<Vec4f> trans,invTrans;
		BBox bBox;
		const BaseTree_ *tree;
	};


#endif


