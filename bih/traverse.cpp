#ifndef RTRACER_BIH_TRAVERSE_H
#define RTRACER_BIH_TRAVERSE_H

#include "bih/tree.h"

namespace bih {

	#include "bih/traverse_mono.h"
	#include "bih/traverse_packet.h"
	#include "bih/traverse_primary.h"

		template<class ElementContainer> template <int flags,template <int> class Selector>
		Isct<f32x4,1,TreeFlags<Tree<ElementContainer>,flags>::value>
			Tree<ElementContainer>::TraversePacket(const RayGroup<1,flags> &rays,const Selector<1> &selector) const
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
						TraverseMono<flags|isctFlags>
						(Vec3p(orig.x[o],orig.y[o],orig.z[o]),Vec3p(dir.x[q],dir.y[q],dir.z[q]),tMaxDist);
					out.Insert(tOut,q);
				}
			}

			return out;
		}

		template<class ElementContainer> template <int flags,int size,template <int> class Selector>
			Isct<f32x4,size,TreeFlags<Tree<ElementContainer>,flags>::value>
			Tree<ElementContainer>::TraversePacket(const RayGroup<size,flags> &rays,const Selector<size> &selector) const
		{
			static_assert(flags&isct::fInvDir,"");

			Isct<f32x4,size,isctFlags|flags> out;	
			bool split=1;

			enum { reflected=!(flags&(isct::fPrimary|isct::fShadow)) };

			bool selectorsFiltered=size<=(reflected?4:16);
			if(!Selector<size>::full)
				for(int n=0;n<size/4;n++)
					if(selector.Mask4(n)!=0x0f0f0f0f) {
						selectorsFiltered=0;
						break;
					}

			if((Selector<size>::full||selectorsFiltered)&&size<=(reflected?4:16)) {
				const Vec3q &dir=rays.Dir(0);
				bool signsFiltered=1;
				int msk=_mm_movemask_ps(_mm_shuffle_ps(_mm_shuffle_ps(dir.x.m,dir.y.m,0),dir.z.m,0+(2<<2)))&7;

				if(filterSigns) {					
					for(int n=0;n<size;n++) if(GetVecSign(rays.Dir(n))!=msk) {
						signsFiltered=0;
						break;
					}
				}

				if(signsFiltered) {
					bool primary=flags&(isct::fPrimary|isct::fShadow)&&gVals[1];

					if(flags&isct::fShadow) {
						floatq dot=1.0f;
						for(int q=1;q<size;q++) dot=Min(dot,rays.Dir(0)|rays.Dir(q));
						if(ForAny(dot<0.9998f)) primary=0;
					}
					out=primary?TraversePrimary(rays):TraversePacket0(rays);

					if(primary&&(flags&isct::fShadow)) out.Stats().Skip();
					split=0;
				}
			}

			if(split) {
				RayGroup<size/4,flags> subGroups[4];
				rays.Split(subGroups);
				subGroups[0].lastShadowTri=rays.lastShadowTri;

				for(int q=0;q<4;q++) {
					Isct<f32x4,size/4,isctFlags|flags> tOut=
						TraversePacket(subGroups[q],selector.SubSelector(q));
					if(q!=3) subGroups[q+1].lastShadowTri=tOut.LastShadowTri();
					out.Insert(tOut,q);
				}

			}

			return out;
		}

}

#include "tree_box.h"

typedef bih::Tree<TriangleVector> StaticTree;
typedef bih::Tree<TreeBoxVector<StaticTree> > FullTree;

enum {
	pFlags=isct::fShOrig|isct::fInvDir|isct::fPrimary,
	sFlags=isct::fShOrig|isct::fInvDir|isct::fMaxDist|isct::fShadow,
};


#define INST(TREE,FLAGS,SIZE,SELECTOR) \
template Isct<f32x4,1,bih::TreeFlags<TREE,FLAGS>::value> \
	TREE::TraversePacket<FLAGS,SELECTOR>(const RayGroup<1,FLAGS>&,const SELECTOR<1>&) const; \
template Isct<f32x4,SIZE,bih::TreeFlags<TREE,FLAGS>::value> \
	TREE::TraversePacket<FLAGS,SIZE,SELECTOR>(const RayGroup<SIZE,FLAGS>&,const SELECTOR<SIZE>&) const;

INST(StaticTree,pFlags,64,FullSelector)
INST(StaticTree,sFlags,64,RaySelector)

INST(FullTree,pFlags,64,FullSelector)
INST(FullTree,sFlags,64,RaySelector)

#undef INST

#endif

