#ifndef RTRACER_BIH_TRAVERSE_H
#define RTRACER_BIH_TRAVERSE_H

#include "bih/tree.h"

namespace bih {

	#include "bih/traverse_mono.h"
	#include "bih/traverse_packet.h"
	#include "bih/traverse_primary.h"

		template<class ElementContainer> template <int flags,template <int> class Selector>
		void 
			Tree<ElementContainer>::TraversePacket(Context<1,flags> &c,const Selector<1> &selector) const
		{
			int bitMask=selector[0];
			const Vec3q &dir=c.Dir(0);

			bool split=1;

			if(Selector<1>::full||bitMask==0x0f) {	
				if(!filterSigns||GetVecSign(c.Dir(0))!=8) {
					TraversePacket0(c);
					split=0;
				}
			}

			if(split) {
				if(bitMask==0) return;
				for(int q=0;q<4;q++) {	
					if(!(bitMask&(1<<q))) continue;

					FContext<flags> fc(c,q);
					TraverseMono(fc);
				} 
			}
		}

		template<class ElementContainer> template <int flags,int size,template <int> class Selector>
			void
			Tree<ElementContainer>::TraversePacket(Context<size,flags> &c,const Selector<size> &selector) const
		{
			bool split=1;

			enum { reflected=!(flags&(isct::fPrimary|isct::fShadow)) };

			bool selectorsFiltered=size<=(reflected?4:isComplex?64:16);
			if(!Selector<size>::full)
				for(int n=0;n<size/4;n++)
					if(selector.Mask4(n)!=0x0f0f0f0f) {
						selectorsFiltered=0;
						break;
					}

			if((Selector<size>::full||selectorsFiltered)&&size<=(reflected?4:isComplex?64:16)) {
				const Vec3q &dir=c.Dir(0);
				bool signsFiltered=1;
				int msk=_mm_movemask_ps(_mm_shuffle_ps(_mm_shuffle_ps(dir.x.m,dir.y.m,0),dir.z.m,0+(2<<2)))&7;

				if(filterSigns) {					
					for(int n=0;n<size;n++) if(GetVecSign(c.Dir(n))!=msk) {
						signsFiltered=0;
						break;
					}
				}

				if(signsFiltered) {
					bool primary=flags&(isct::fPrimary|isct::fShadow)&&gVals[1];

					if(flags&isct::fShadow&&!isComplex) {
						floatq dot=1.0f;
						for(int q=1;q<size;q++) dot=Min(dot,c.Dir(0)|c.Dir(q));
						if(ForAny(dot<0.9998f)) primary=0;
					}
					if(primary) TraversePrimary(c);
					else TraversePacket0(c);

				//	if(primary&&(flags&isct::fShadow)) c.stats.Skip();
					split=0;
				}
			}

			if(split) {
				for(int q=0;q<4;q++) {
					Context<size/4,flags> subC(c.Split(q));
					if(flags&isct::fShadow) subC.shadowCache=c.shadowCache;
					TraversePacket(subC,selector.SubSelector(q));
					if(flags&isct::fShadow) c.shadowCache=subC.shadowCache;
				}
			}
		}

}

#include "tree_box.h"

typedef bih::Tree<TriangleVector> StaticTree;
typedef bih::Tree<TreeBoxVector<StaticTree> > FullTree;

enum {
	pFlags=isct::fShOrig|isct::fPrimary,
	sFlags=isct::fShOrig|isct::fShadow,
	rFlags=0,
};


#define INST(TREE,FLAGS,SIZE,SELECTOR) \
template void TREE::TraversePacket<FLAGS,SELECTOR>(Context<1,FLAGS>&,const SELECTOR<1>&) const; \
template void TREE::TraversePacket<FLAGS,SIZE,SELECTOR>(Context<SIZE,FLAGS>&,const SELECTOR<SIZE>&) const;

INST(StaticTree,pFlags,64,FullSelector)
INST(StaticTree,sFlags,64,RaySelector)
INST(StaticTree,rFlags,64,RaySelector)

INST(FullTree,pFlags,64,FullSelector)
INST(FullTree,sFlags,64,RaySelector)
INST(FullTree,rFlags,64,RaySelector)

#undef INST

#endif

